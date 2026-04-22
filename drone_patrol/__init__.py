"""Drone patrol fleet management primitives."""

from .fleet_state import (
    DroneCommand,
    DroneCommandType,
    DroneState,
    DroneStatus,
    FleetPolicy,
    FleetStateManager,
    GPSPosition,
    RotationDecision,
)
from .patrol_routes import MAVLinkMissionWaypoint, generate_perimeter_patrol_route
from .patrol_routes import generate_coverage_sweep_route, generate_grid_survey_route
from .telemetry_receiver import DroneTelemetryReceiver, TelemetrySnapshot, default_status_resolver

__all__ = [
    "DroneCommand",
    "DroneCommandType",
    "DroneState",
    "DroneStatus",
    "FleetPolicy",
    "FleetStateManager",
    "GPSPosition",
    "MAVLinkMissionWaypoint",
    "RotationDecision",
    "DroneTelemetryReceiver",
    "TelemetrySnapshot",
    "default_status_resolver",
    "generate_coverage_sweep_route",
    "generate_grid_survey_route",
    "generate_perimeter_patrol_route",
]
