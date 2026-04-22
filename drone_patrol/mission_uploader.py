from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

from .patrol_routes import MAVLinkMissionWaypoint

VehicleConnector = Callable[..., Any]
CommandFactory = Callable[..., Any]
VehicleModeFactory = Callable[[str], Any]


def upload_mission(
    *,
    connection_string: str,
    mission_waypoints: Sequence[dict[str, int | float]],
    baud: int = 57600,
    connect_timeout_s: float = 30.0,
    wait_ready: bool = True,
    connect_callable: VehicleConnector | None = None,
    command_factory: CommandFactory | None = None,
    vehicle_mode_factory: VehicleModeFactory | None = None,
    connect_kwargs: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if not mission_waypoints:
        raise ValueError("mission_waypoints must contain at least one waypoint.")

    connect_callable = connect_callable or _default_connect
    command_factory = command_factory or _default_command_factory
    vehicle_mode_factory = vehicle_mode_factory or _default_vehicle_mode_factory

    connect_options = {
        "wait_ready": wait_ready,
        "timeout": connect_timeout_s,
        "heartbeat_timeout": connect_timeout_s,
        **(connect_kwargs or {}),
    }
    if baud:
        connect_options.setdefault("baud", baud)

    vehicle = connect_callable(connection_string, **connect_options)
    try:
        commands = getattr(vehicle, "commands", None)
        if commands is None:
            raise RuntimeError("Connected vehicle does not expose a mission command interface.")

        download_method = getattr(commands, "download", None)
        wait_ready_method = getattr(commands, "wait_ready", None)
        if callable(download_method):
            download_method()
        if callable(wait_ready_method):
            wait_ready_method()

        clear_method = getattr(commands, "clear", None)
        if not callable(clear_method):
            raise RuntimeError("Connected vehicle does not support mission clearing.")
        clear_method()

        uploaded_count = 0
        for item in mission_waypoints:
            waypoint = MAVLinkMissionWaypoint.model_validate(item)
            add_method = getattr(commands, "add", None)
            if not callable(add_method):
                raise RuntimeError("Connected vehicle does not support mission upload.")
            add_method(
                command_factory(
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
                    waypoint.x_lat,
                    waypoint.y_lon,
                    waypoint.z_alt,
                )
            )
            uploaded_count += 1

        upload_method = getattr(commands, "upload", None)
        if not callable(upload_method):
            raise RuntimeError("Connected vehicle does not support mission upload finalization.")
        upload_method()

        vehicle.mode = vehicle_mode_factory("AUTO")
        flush_method = getattr(vehicle, "flush", None)
        if callable(flush_method):
            flush_method()

        return {
            "connection_string": connection_string,
            "uploaded_waypoints": uploaded_count,
            "mode": "AUTO",
        }
    finally:
        close_method = getattr(vehicle, "close", None)
        if callable(close_method):
            close_method()


def _default_connect(connection_string: str, **kwargs: Any) -> Any:
    try:
        from dronekit import connect
    except ImportError as exc:
        raise ImportError(
            "DroneKit is required for mission upload connections. Install it with `pip install dronekit`."
        ) from exc
    return connect(connection_string, **kwargs)


def _default_command_factory(*args: Any) -> Any:
    try:
        from dronekit import Command
    except ImportError as exc:
        raise ImportError(
            "DroneKit is required for mission upload commands. Install it with `pip install dronekit`."
        ) from exc
    return Command(*args)


def _default_vehicle_mode_factory(mode_name: str) -> Any:
    try:
        from dronekit import VehicleMode
    except ImportError as exc:
        raise ImportError(
            "DroneKit is required for mission upload mode changes. Install it with `pip install dronekit`."
        ) from exc
    return VehicleMode(mode_name)
