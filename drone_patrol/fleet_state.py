from __future__ import annotations

from datetime import datetime, timezone
from enum import StrEnum
from threading import RLock

from pydantic import BaseModel, Field


def utcnow() -> datetime:
    return datetime.now(timezone.utc)


class DroneStatus(StrEnum):
    FLYING = "flying"
    CHARGING = "charging"
    STANDBY = "standby"
    FAILED = "failed"


class DroneCommandType(StrEnum):
    RECALL_TO_CHARGER = "recall_to_charger"
    LAUNCH_TO_PATROL = "launch_to_patrol"


class GPSPosition(BaseModel):
    latitude: float = Field(ge=-90.0, le=90.0)
    longitude: float = Field(ge=-180.0, le=180.0)
    altitude_m: float | None = None


class DroneState(BaseModel):
    drone_id: str
    battery_level: float = Field(ge=0.0, le=100.0)
    status: DroneStatus
    position: GPSPosition | None = None
    heading_deg: float | None = Field(default=None, ge=0.0, le=360.0)
    flight_mode: str | None = None
    armed: bool | None = None
    updated_at: datetime = Field(default_factory=utcnow)
    failure_reason: str | None = None

    def is_launch_ready(self, minimum_launch_battery: float) -> bool:
        return (
            self.status == DroneStatus.STANDBY
            and self.failure_reason is None
            and self.battery_level >= minimum_launch_battery
        )


class FleetPolicy(BaseModel):
    recall_battery_threshold: float = Field(default=25.0, ge=0.0, le=100.0)
    minimum_launch_battery: float = Field(default=90.0, ge=0.0, le=100.0)
    target_airborne_drones: int = Field(default=1, ge=0)


class DroneCommand(BaseModel):
    command: DroneCommandType
    drone_id: str
    reason: str


class RotationDecision(BaseModel):
    evaluated_at: datetime = Field(default_factory=utcnow)
    commands: list[DroneCommand] = Field(default_factory=list)
    active_drone_ids: list[str] = Field(default_factory=list)
    ready_standby_ids: list[str] = Field(default_factory=list)
    unmet_airborne_slots: int = 0

    @property
    def has_commands(self) -> bool:
        return bool(self.commands)


class FleetStateManager:
    """Tracks drone state and produces rotation commands from live telemetry."""

    def __init__(
        self,
        drones: list[DroneState] | None = None,
        policy: FleetPolicy | None = None,
    ) -> None:
        self._policy = policy or FleetPolicy()
        self._drones: dict[str, DroneState] = {}
        self._lock = RLock()
        for drone in drones or []:
            self.register_drone(drone)

    @property
    def policy(self) -> FleetPolicy:
        return self._policy.model_copy(deep=True)

    def register_drone(self, drone: DroneState) -> DroneState:
        with self._lock:
            self._drones[drone.drone_id] = drone.model_copy(deep=True)
            return self.get_drone(drone.drone_id)

    def get_drone(self, drone_id: str) -> DroneState:
        with self._lock:
            if drone_id not in self._drones:
                raise KeyError(f"Unknown drone '{drone_id}'")
            return self._drones[drone_id].model_copy(deep=True)

    def list_drones(self, status: DroneStatus | None = None) -> list[DroneState]:
        with self._lock:
            drones = self._drones.values()
            if status is not None:
                drones = (drone for drone in drones if drone.status == status)
            return [drone.model_copy(deep=True) for drone in drones]

    def snapshot(self) -> list[DroneState]:
        return self.list_drones()

    def upsert_drone(
        self,
        drone_id: str,
        *,
        battery_level: float,
        status: DroneStatus,
        position: GPSPosition | None = None,
        heading_deg: float | None = None,
        flight_mode: str | None = None,
        armed: bool | None = None,
        updated_at: datetime | None = None,
        failure_reason: str | None = None,
    ) -> DroneState:
        drone = DroneState(
            drone_id=drone_id,
            battery_level=battery_level,
            status=status,
            position=position,
            heading_deg=heading_deg,
            flight_mode=flight_mode,
            armed=armed,
            updated_at=updated_at or utcnow(),
            failure_reason=failure_reason,
        )
        with self._lock:
            self._drones[drone_id] = drone
            return drone.model_copy(deep=True)

    def update_drone(
        self,
        drone_id: str,
        *,
        battery_level: float | None = None,
        status: DroneStatus | None = None,
        position: GPSPosition | None = None,
        heading_deg: float | None = None,
        flight_mode: str | None = None,
        armed: bool | None = None,
        updated_at: datetime | None = None,
        failure_reason: str | None = None,
    ) -> DroneState:
        current = self.get_drone(drone_id)
        next_position = current.position if position is None else position
        next_heading = current.heading_deg if heading_deg is None else heading_deg
        next_mode = current.flight_mode if flight_mode is None else flight_mode
        next_armed = current.armed if armed is None else armed
        next_failure_reason = current.failure_reason if failure_reason is None else failure_reason

        updated = current.model_copy(
            update={
                "battery_level": current.battery_level if battery_level is None else battery_level,
                "status": current.status if status is None else status,
                "position": next_position,
                "heading_deg": next_heading,
                "flight_mode": next_mode,
                "armed": next_armed,
                "updated_at": updated_at or utcnow(),
                "failure_reason": next_failure_reason,
            }
        )
        with self._lock:
            self._drones[drone_id] = updated
            return updated.model_copy(deep=True)

    def mark_failed(
        self,
        drone_id: str,
        *,
        failure_reason: str,
        position: GPSPosition | None = None,
        updated_at: datetime | None = None,
    ) -> DroneState:
        return self.update_drone(
            drone_id,
            status=DroneStatus.FAILED,
            position=position,
            updated_at=updated_at,
            failure_reason=failure_reason,
        )

    def evaluate_rotation(self, policy: FleetPolicy | None = None) -> RotationDecision:
        effective_policy = policy or self._policy
        with self._lock:
            drones = [drone.model_copy(deep=True) for drone in self._drones.values()]
        flying = sorted(
            (drone for drone in drones if drone.status == DroneStatus.FLYING),
            key=lambda drone: (drone.battery_level, drone.updated_at),
        )
        standby_ready = sorted(
            (
                drone
                for drone in drones
                if drone.is_launch_ready(effective_policy.minimum_launch_battery)
            ),
            key=lambda drone: (-drone.battery_level, drone.updated_at, drone.drone_id),
        )

        commands: list[DroneCommand] = []
        recall_ids: set[str] = set()
        for drone in flying:
            if drone.battery_level <= effective_policy.recall_battery_threshold:
                recall_ids.add(drone.drone_id)
                commands.append(
                    DroneCommand(
                        command=DroneCommandType.RECALL_TO_CHARGER,
                        drone_id=drone.drone_id,
                        reason=(
                            f"Battery at {drone.battery_level:.1f}% is at or below "
                            f"the {effective_policy.recall_battery_threshold:.1f}% recall threshold."
                        ),
                    )
                )

        active_after_recalls = [drone.drone_id for drone in flying if drone.drone_id not in recall_ids]
        launch_slots = max(effective_policy.target_airborne_drones - len(active_after_recalls), 0)
        launch_candidates = standby_ready[:launch_slots]

        for drone in launch_candidates:
            commands.append(
                DroneCommand(
                    command=DroneCommandType.LAUNCH_TO_PATROL,
                    drone_id=drone.drone_id,
                    reason=(
                        f"Standby drone is ready with {drone.battery_level:.1f}% battery "
                        f"to maintain {effective_policy.target_airborne_drones} airborne drone(s)."
                    ),
                )
            )
            active_after_recalls.append(drone.drone_id)

        ready_standby_ids = [drone.drone_id for drone in standby_ready]
        unmet_airborne_slots = max(effective_policy.target_airborne_drones - len(active_after_recalls), 0)

        return RotationDecision(
            commands=commands,
            active_drone_ids=active_after_recalls,
            ready_standby_ids=ready_standby_ids,
            unmet_airborne_slots=unmet_airborne_slots,
        )
