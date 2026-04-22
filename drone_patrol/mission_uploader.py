from __future__ import annotations

import collections
import collections.abc
import traceback
from time import monotonic, sleep
from collections.abc import Callable, Sequence
from typing import Any

collections.MutableMapping = collections.abc.MutableMapping
collections.MutableSequence = collections.abc.MutableSequence

from .patrol_routes import MAVLinkMissionWaypoint

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
            for item in mission_waypoints:
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
            print("[mission_uploader] Arming vehicle after mission upload")
            vehicle.armed = True
            flush_method = getattr(vehicle, "flush", None)
            if callable(flush_method):
                flush_method()
            if not _wait_for_condition(lambda: bool(getattr(vehicle, "armed", False)), arm_timeout_s):
                raise RuntimeError("Vehicle did not report armed state after upload.")

            vehicle.mode = vehicle_mode_factory("AUTO")
            if callable(flush_method):
                flush_method()
            if not _wait_for_condition(lambda: _vehicle_mode_name(vehicle) == "AUTO", mode_change_timeout_s):
                raise RuntimeError(f"Vehicle mode remained {_vehicle_mode_name(vehicle)!r} instead of 'AUTO'.")
            print(f"[mission_uploader] Vehicle armed={getattr(vehicle, 'armed', None)} mode={_vehicle_mode_name(vehicle)}")
        except Exception as exc:
            _log_upload_exception("setting AUTO mode", exc)
            raise MissionUploadError("setting AUTO mode", exc) from exc

        return {
            "connection_string": connection_string,
            "uploaded_waypoints": uploaded_count,
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


def _vehicle_mode_name(vehicle: Any) -> str | None:
    mode = getattr(vehicle, "mode", None)
    if mode is None:
        return None
    return getattr(mode, "name", mode)


def _wait_for_condition(predicate: Callable[[], bool], timeout_s: float, poll_interval_s: float = 0.2) -> bool:
    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        if predicate():
            return True
        sleep(poll_interval_s)
    return predicate()
