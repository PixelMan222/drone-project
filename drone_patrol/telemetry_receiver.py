from __future__ import annotations

import collections
import collections.abc
from collections.abc import Callable
from dataclasses import dataclass
from threading import Event, Thread
from typing import Any

collections.MutableMapping = collections.abc.MutableMapping
collections.MutableSequence = collections.abc.MutableSequence

from .fleet_state import DroneState, DroneStatus, FleetStateManager, GPSPosition, utcnow


VehicleConnector = Callable[..., Any]
StatusResolver = Callable[["TelemetrySnapshot", DroneState | None], DroneStatus]


@dataclass(slots=True)
class TelemetrySnapshot:
    drone_id: str
    battery_level: float | None
    position: GPSPosition | None
    heading_deg: float | None
    flight_mode: str | None
    armed: bool | None


class DroneTelemetryReceiver:
    """Background DroneKit telemetry reader for a single drone connection."""

    def __init__(
        self,
        *,
        drone_id: str,
        connection_string: str,
        fleet_manager: FleetStateManager,
        baud: int = 57600,
        poll_interval_s: float = 1.0,
        connect_timeout_s: float = 30.0,
        wait_ready: bool = True,
        connect_callable: VehicleConnector | None = None,
        status_resolver: StatusResolver | None = None,
        connect_kwargs: dict[str, Any] | None = None,
        daemon: bool = True,
    ) -> None:
        self._drone_id = drone_id
        self._connection_string = connection_string
        self._fleet_manager = fleet_manager
        self._baud = baud
        self._poll_interval_s = poll_interval_s
        self._connect_timeout_s = connect_timeout_s
        self._wait_ready = wait_ready
        self._connect_callable = connect_callable or _default_connect
        self._status_resolver = status_resolver or default_status_resolver
        self._connect_kwargs = connect_kwargs or {}
        self._stop_event = Event()
        self._thread = Thread(target=self._run, name=f"telemetry-{drone_id}", daemon=daemon)
        self._vehicle: Any | None = None
        self._last_error: Exception | None = None

    @property
    def last_error(self) -> Exception | None:
        return self._last_error

    @property
    def is_running(self) -> bool:
        return self._thread.is_alive()

    def start(self) -> None:
        if self._thread.is_alive():
            return
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def join(self, timeout: float | None = None) -> None:
        self._thread.join(timeout=timeout)

    def _run(self) -> None:
        try:
            self._vehicle = self._connect_vehicle()
            self._telemetry_loop()
        except Exception as exc:
            self._last_error = exc
            self._mark_failed(str(exc))
        finally:
            self._close_vehicle()

    def _connect_vehicle(self) -> Any:
        connect_kwargs = {
            "wait_ready": self._wait_ready,
            "timeout": self._connect_timeout_s,
            "heartbeat_timeout": self._connect_timeout_s,
            **self._connect_kwargs,
        }
        if self._baud:
            connect_kwargs.setdefault("baud", self._baud)
        return self._connect_callable(self._connection_string, **connect_kwargs)

    def _telemetry_loop(self) -> None:
        while not self._stop_event.is_set():
            snapshot = self._read_snapshot()
            self._apply_snapshot(snapshot)
            if self._stop_event.wait(self._poll_interval_s):
                break

    def _read_snapshot(self) -> TelemetrySnapshot:
        vehicle = self._vehicle
        if vehicle is None:
            raise RuntimeError("Telemetry vehicle connection is not available.")

        location = getattr(getattr(vehicle, "location", None), "global_relative_frame", None)
        position = None
        if location is not None and getattr(location, "lat", None) is not None and getattr(location, "lon", None) is not None:
            position = GPSPosition(
                latitude=float(location.lat),
                longitude=float(location.lon),
                altitude_m=_safe_float(getattr(location, "alt", None)),
            )

        battery_level = _extract_battery_level(getattr(vehicle, "battery", None))
        heading_deg = _normalize_heading(getattr(vehicle, "heading", None))
        mode = getattr(getattr(vehicle, "mode", None), "name", None)
        armed = _safe_bool(getattr(vehicle, "armed", None))

        return TelemetrySnapshot(
            drone_id=self._drone_id,
            battery_level=battery_level,
            position=position,
            heading_deg=heading_deg,
            flight_mode=str(mode) if mode is not None else None,
            armed=armed,
        )

    def _apply_snapshot(self, snapshot: TelemetrySnapshot) -> None:
        try:
            current = self._fleet_manager.get_drone(snapshot.drone_id)
        except KeyError:
            current = None

        battery_level = snapshot.battery_level
        if battery_level is None:
            battery_level = current.battery_level if current is not None else 0.0

        status = self._status_resolver(snapshot, current)
        failure_reason = None if status != DroneStatus.FAILED else "Telemetry resolver reported failure."

        if current is None:
            self._fleet_manager.upsert_drone(
                snapshot.drone_id,
                battery_level=battery_level,
                status=status,
                position=snapshot.position,
                heading_deg=snapshot.heading_deg,
                flight_mode=snapshot.flight_mode,
                armed=snapshot.armed,
                updated_at=utcnow(),
                failure_reason=failure_reason,
            )
            return

        self._fleet_manager.update_drone(
            snapshot.drone_id,
            battery_level=battery_level,
            status=status,
            position=snapshot.position,
            heading_deg=snapshot.heading_deg,
            flight_mode=snapshot.flight_mode,
            armed=snapshot.armed,
            updated_at=utcnow(),
            failure_reason=failure_reason,
        )

    def _mark_failed(self, reason: str) -> None:
        try:
            current = self._fleet_manager.get_drone(self._drone_id)
        except KeyError:
            self._fleet_manager.upsert_drone(
                self._drone_id,
                battery_level=0.0,
                status=DroneStatus.FAILED,
                updated_at=utcnow(),
                failure_reason=reason,
            )
            return

        self._fleet_manager.update_drone(
            self._drone_id,
            status=DroneStatus.FAILED,
            updated_at=utcnow(),
            failure_reason=reason,
            position=current.position,
            battery_level=current.battery_level,
            heading_deg=current.heading_deg,
            flight_mode=current.flight_mode,
            armed=current.armed,
        )

    def _close_vehicle(self) -> None:
        if self._vehicle is None:
            return
        close_method = getattr(self._vehicle, "close", None)
        if callable(close_method):
            close_method()
        self._vehicle = None


def default_status_resolver(snapshot: TelemetrySnapshot, current: DroneState | None) -> DroneStatus:
    mode = (snapshot.flight_mode or "").upper()
    if current is not None and current.status == DroneStatus.CHARGING and snapshot.armed is False:
        return DroneStatus.CHARGING
    if mode in {"CHARGING", "DOCK", "DOCKED"}:
        return DroneStatus.CHARGING
    if snapshot.armed:
        return DroneStatus.FLYING
    return DroneStatus.STANDBY


def _default_connect(connection_string: str, **kwargs: Any) -> Any:
    try:
        from dronekit import connect
    except ImportError as exc:
        raise ImportError(
            "DroneKit is required for telemetry connections. Install it with `pip install dronekit`."
        ) from exc
    return connect(connection_string, **kwargs)


def _extract_battery_level(battery: Any) -> float | None:
    level = getattr(battery, "level", None)
    if level is None:
        return None
    parsed = float(level)
    return max(0.0, min(parsed, 100.0))


def _normalize_heading(value: Any) -> float | None:
    heading = _safe_float(value)
    if heading is None:
        return None
    return heading % 360.0


def _safe_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _safe_bool(value: Any) -> bool | None:
    if value is None:
        return None
    return bool(value)
