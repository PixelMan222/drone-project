from __future__ import annotations

import collections
import collections.abc
import traceback
from time import monotonic, sleep
from collections.abc import Callable, Sequence
from typing import Any

collections.MutableMapping = collections.abc.MutableMapping
collections.MutableSequence = collections.abc.MutableSequence

from .patrol_routes import MAVLinkMissionWaypoint, append_loop_jump_to_mission

VehicleConnector = Callable[..., Any]
MissionItemIntFactory = Callable[..., Any]
VehicleModeFactory = Callable[[str], Any]


class MissionUploadError(RuntimeError):
    def __init__(self, step: str, original_error: Exception) -> None:
        self.step = step
        self.original_error = original_error
        super().__init__(f"Mission upload failed during {step}: {original_error}")


def upload_mission(
    *,
    connection_string: str,
    mission_waypoints: Sequence[dict[str, int | float]],
    baud: int = 57600,
    connect_timeout_s: float = 30.0,
    wait_ready: bool = True,
    mode_change_timeout_s: float = 10.0,
    arm_timeout_s: float = 10.0,
    takeoff_altitude_m: float = 10.0,
    takeoff_confirm_altitude_m: float = 5.0,
    takeoff_timeout_s: float = 10.0,
    loop_mission: bool = True,
    loop_to_seq: int = 1,
    connect_callable: VehicleConnector | None = None,
    mission_item_int_factory: MissionItemIntFactory | None = None,
    vehicle_mode_factory: VehicleModeFactory | None = None,
    connect_kwargs: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if not mission_waypoints:
        raise ValueError("mission_waypoints must contain at least one waypoint.")

    connect_callable = connect_callable or _default_connect
    vehicle_mode_factory = vehicle_mode_factory or _default_vehicle_mode_factory

    connect_options = {
        "wait_ready": wait_ready,
        "timeout": connect_timeout_s,
        "heartbeat_timeout": connect_timeout_s,
        **(connect_kwargs or {}),
    }
    if baud:
        connect_options.setdefault("baud", baud)

    try:
        vehicle = connect_callable(connection_string, **connect_options)
    except Exception as exc:
        _log_upload_exception("connecting to drone", exc)
        raise MissionUploadError("connecting to drone", exc) from exc

    try:
        commands = getattr(vehicle, "commands", None)
        if commands is None:
            raise RuntimeError("Connected vehicle does not expose a mission command interface.")
        item_factory = mission_item_int_factory or _default_mission_item_int_factory(vehicle)
        mission_items_to_upload = (
            append_loop_jump_to_mission(mission_waypoints, jump_to_seq=loop_to_seq)
            if loop_mission and len(mission_waypoints) >= 2
            else [dict(item) for item in mission_waypoints]
        )

        download_method = getattr(commands, "download", None)
        wait_ready_method = getattr(commands, "wait_ready", None)
        if callable(download_method):
            download_method()
        if callable(wait_ready_method):
            wait_ready_method()

        clear_method = getattr(commands, "clear", None)
        if not callable(clear_method):
            raise RuntimeError("Connected vehicle does not support mission clearing.")
        try:
            clear_method()
        except Exception as exc:
            _log_upload_exception("clearing mission", exc)
            raise MissionUploadError("clearing mission", exc) from exc

        uploaded_count = 0
        try:
            for item in mission_items_to_upload:
                waypoint = MAVLinkMissionWaypoint.model_validate(item)
                add_method = getattr(commands, "add", None)
                if not callable(add_method):
                    raise RuntimeError("Connected vehicle does not support mission upload.")
                add_method(
                    item_factory(
                        0,
                        0,
                        waypoint.seq,
                        waypoint.frame,
                        waypoint.command,
                        waypoint.current,
                        waypoint.autocontinue,
                        waypoint.param1,
                        waypoint.param2,
                        waypoint.param3,
                        waypoint.param4,
                        int(round(float(waypoint.x_lat) * 1e7)),
                        int(round(float(waypoint.y_lon) * 1e7)),
                        waypoint.z_alt,
                    )
                )
                uploaded_count += 1

            upload_method = getattr(commands, "upload", None)
            if not callable(upload_method):
                raise RuntimeError("Connected vehicle does not support mission upload finalization.")
            upload_method()
        except Exception as exc:
            _log_upload_exception("uploading waypoints", exc)
            raise MissionUploadError("uploading waypoints", exc) from exc

        try:
            flush_method = getattr(vehicle, "flush", None)
            print("[mission_uploader] Disabling ARMING_CHECK preflight gate")
            _disable_arming_checks(vehicle)
            if callable(flush_method):
                flush_method()

            print("[mission_uploader] Switching vehicle to GUIDED before arming")
            vehicle.mode = vehicle_mode_factory("GUIDED")
            if callable(flush_method):
                flush_method()
            if not _wait_for_condition(lambda: _vehicle_mode_name(vehicle) == "GUIDED", mode_change_timeout_s):
                raise RuntimeError(
                    f"Vehicle mode remained {_vehicle_mode_name(vehicle)!r} instead of 'GUIDED' before arming."
                )

            print("[mission_uploader] Arming vehicle in GUIDED mode")
            vehicle.armed = True
            if callable(flush_method):
                flush_method()
            if not _wait_for_condition(lambda: bool(getattr(vehicle, "armed", False)), arm_timeout_s):
                raise RuntimeError(f"Vehicle did not report armed within {arm_timeout_s:.1f}s. {_arm_failure_reason(vehicle)}")
            print("[mission_uploader] Vehicle confirmed armed")
        except Exception as exc:
            _log_upload_exception("arming vehicle", exc)
            raise MissionUploadError("arming vehicle", exc) from exc

        try:
            flush_method = getattr(vehicle, "flush", None)
            takeoff_method = getattr(vehicle, "simple_takeoff", None)
            if not callable(takeoff_method):
                raise RuntimeError("Connected vehicle does not support simple_takeoff for launch sequencing.")
            print(f"[mission_uploader] Commanding takeoff to {takeoff_altitude_m:.1f} meters")
            takeoff_method(takeoff_altitude_m)
            if callable(flush_method):
                flush_method()
            reached_takeoff_altitude = _wait_for_condition(
                lambda: (_vehicle_relative_altitude(vehicle) or 0.0) >= takeoff_confirm_altitude_m,
                takeoff_timeout_s,
            )
            if reached_takeoff_altitude:
                print(
                    "[mission_uploader] Vehicle reached "
                    f"{_vehicle_relative_altitude(vehicle) or 0.0:.1f} meters relative altitude before AUTO"
                )
            else:
                print(
                    "[mission_uploader] Warning: vehicle did not reach "
                    f"{takeoff_confirm_altitude_m:.1f} meters within {takeoff_timeout_s:.1f}s; "
                    f"current altitude={(_vehicle_relative_altitude(vehicle) or 0.0):.1f}m. Proceeding to AUTO."
                )
        except Exception as exc:
            _log_upload_exception("commanding takeoff", exc)
            raise MissionUploadError("commanding takeoff", exc) from exc

        try:
            flush_method = getattr(vehicle, "flush", None)
            print("[mission_uploader] Switching vehicle to AUTO mode")
            vehicle.mode = vehicle_mode_factory("AUTO")
            if callable(flush_method):
                flush_method()
            if not _wait_for_condition(lambda: _vehicle_mode_name(vehicle) == "AUTO", mode_change_timeout_s):
                raise RuntimeError(f"Vehicle mode remained {_vehicle_mode_name(vehicle)!r} instead of 'AUTO'.")
            print(
                f"[mission_uploader] Vehicle armed={getattr(vehicle, 'armed', None)} mode={_vehicle_mode_name(vehicle)}"
            )
        except Exception as exc:
            _log_upload_exception("setting AUTO mode", exc)
            raise MissionUploadError("setting AUTO mode", exc) from exc

        return {
            "connection_string": connection_string,
            "uploaded_waypoints": len(mission_waypoints),
            "uploaded_mission_items": uploaded_count,
            "loop_enabled": loop_mission and len(mission_waypoints) >= 2,
            "mode": _vehicle_mode_name(vehicle),
            "armed": bool(getattr(vehicle, "armed", False)),
        }
    finally:
        close_method = getattr(vehicle, "close", None)
        if callable(close_method):
            close_method()


def _log_upload_exception(step: str, exc: Exception) -> None:
    print(f"[mission_uploader] Mission upload failed during {step}: {exc}")
    traceback.print_exc()


def _default_connect(connection_string: str, **kwargs: Any) -> Any:
    try:
        from dronekit import connect
    except ImportError as exc:
        raise ImportError(
            "DroneKit is required for mission upload connections. Install it with `pip install dronekit`."
        ) from exc
    return connect(connection_string, **kwargs)


def _default_mission_item_int_factory(vehicle: Any) -> MissionItemIntFactory:
    message_factory = getattr(vehicle, "message_factory", None)
    encoder = getattr(message_factory, "mission_item_int_encode", None) if message_factory is not None else None
    if not callable(encoder):
        raise RuntimeError("Connected vehicle does not expose mission_item_int_encode for mission upload.")
    return encoder


def _default_vehicle_mode_factory(mode_name: str) -> Any:
    try:
        from dronekit import VehicleMode
    except ImportError as exc:
        raise ImportError(
            "DroneKit is required for mission upload mode changes. Install it with `pip install dronekit`."
        ) from exc
    return VehicleMode(mode_name)


def set_vehicle_mode(
    *,
    connection_string: str,
    target_mode: str,
    baud: int = 57600,
    connect_timeout_s: float = 30.0,
    wait_ready: bool = True,
    mode_change_timeout_s: float = 10.0,
    connect_callable: VehicleConnector | None = None,
    vehicle_mode_factory: VehicleModeFactory | None = None,
    connect_kwargs: dict[str, Any] | None = None,
) -> dict[str, Any]:
    connect_callable = connect_callable or _default_connect
    vehicle_mode_factory = vehicle_mode_factory or _default_vehicle_mode_factory

    connect_options = {
        "wait_ready": wait_ready,
        "timeout": connect_timeout_s,
        "heartbeat_timeout": connect_timeout_s,
        **(connect_kwargs or {}),
    }
    if baud:
        connect_options.setdefault("baud", baud)

    try:
        vehicle = connect_callable(connection_string, **connect_options)
    except Exception as exc:
        _log_upload_exception("connecting to drone", exc)
        raise RuntimeError(f"Failed to connect to drone for {target_mode} mode command: {exc}") from exc

    try:
        flush_method = getattr(vehicle, "flush", None)
        print(f"[mission_uploader] Setting vehicle mode to {target_mode}")
        vehicle.mode = vehicle_mode_factory(target_mode)
        if callable(flush_method):
            flush_method()
        if not _wait_for_condition(lambda: _vehicle_mode_name(vehicle) == target_mode, mode_change_timeout_s):
            raise RuntimeError(f"Vehicle mode remained {_vehicle_mode_name(vehicle)!r} instead of {target_mode!r}.")
        print(f"[mission_uploader] Vehicle mode confirmed as {target_mode}")
        return {
            "connection_string": connection_string,
            "mode": _vehicle_mode_name(vehicle),
            "armed": bool(getattr(vehicle, "armed", False)),
        }
    except Exception as exc:
        _log_upload_exception(f"setting {target_mode} mode", exc)
        raise RuntimeError(f"Failed to set drone mode to {target_mode}: {exc}") from exc
    finally:
        close_method = getattr(vehicle, "close", None)
        if callable(close_method):
            close_method()


def _disable_arming_checks(vehicle: Any) -> None:
    parameters = getattr(vehicle, "parameters", None)
    if parameters is not None:
        try:
            parameters["ARMING_CHECK"] = 0
            print("[mission_uploader] ARMING_CHECK parameter set to 0 via vehicle.parameters")
            return
        except Exception:
            pass

    message_factory = getattr(vehicle, "message_factory", None)
    param_set_encoder = getattr(message_factory, "param_set_encode", None) if message_factory is not None else None
    send_mavlink = getattr(vehicle, "send_mavlink", None)
    if callable(param_set_encoder) and callable(send_mavlink):
        mavutil = _import_mavutil()
        command = param_set_encoder(
            0,
            0,
            b"ARMING_CHECK",
            0.0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        send_mavlink(command)
        print("[mission_uploader] ARMING_CHECK parameter set to 0 via MAVLink PARAM_SET")
        return

    raise RuntimeError("Connected vehicle does not support disabling ARMING_CHECK before arming.")


def _import_mavutil() -> Any:
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise ImportError(
            "pymavlink is required to disable ARMING_CHECK via MAVLink PARAM_SET."
        ) from exc
    return mavutil


def _vehicle_mode_name(vehicle: Any) -> str | None:
    mode = getattr(vehicle, "mode", None)
    if mode is None:
        return None
    return getattr(mode, "name", mode)


def _vehicle_relative_altitude(vehicle: Any) -> float | None:
    location = getattr(vehicle, "location", None)
    if location is None:
        return None
    relative_frame = getattr(location, "global_relative_frame", None)
    if relative_frame is None:
        return None
    altitude = getattr(relative_frame, "alt", None)
    if altitude is None:
        return None
    try:
        return float(altitude)
    except (TypeError, ValueError):
        return None


def _arm_failure_reason(vehicle: Any) -> str:
    details: list[str] = []

    mode_name = _vehicle_mode_name(vehicle)
    if mode_name is not None:
        details.append(f"mode={mode_name}")

    if hasattr(vehicle, "is_armable"):
        details.append(f"is_armable={getattr(vehicle, 'is_armable', None)}")

    system_status = getattr(vehicle, "system_status", None)
    if system_status is not None:
        details.append(f"system_status={getattr(system_status, 'state', system_status)}")

    gps = getattr(vehicle, "gps_0", None)
    if gps is not None:
        fix_type = getattr(gps, "fix_type", None)
        satellites_visible = getattr(gps, "satellites_visible", None)
        gps_bits: list[str] = []
        if fix_type is not None:
            gps_bits.append(f"fix_type={fix_type}")
        if satellites_visible is not None:
            gps_bits.append(f"satellites_visible={satellites_visible}")
        if gps_bits:
            details.append("gps(" + ", ".join(gps_bits) + ")")

    last_heartbeat = getattr(vehicle, "last_heartbeat", None)
    if last_heartbeat is not None:
        details.append(f"last_heartbeat={last_heartbeat}")

    if not details:
        return "No additional arming diagnostics were available from the vehicle."
    return "Arming diagnostics: " + ", ".join(details)


def _wait_for_condition(predicate: Callable[[], bool], timeout_s: float, poll_interval_s: float = 0.2) -> bool:
    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        if predicate():
            return True
        sleep(poll_interval_s)
    return predicate()
