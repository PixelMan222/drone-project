from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Any

try:
    from flask import Flask, jsonify, request, send_from_directory
    from flask_socketio import SocketIO
except ImportError:  # pragma: no cover
    Flask = None  # type: ignore[assignment]
    SocketIO = None  # type: ignore[assignment]
    jsonify = None  # type: ignore[assignment]
    request = None  # type: ignore[assignment]
    send_from_directory = None  # type: ignore[assignment]

from drone_patrol.fleet_state import DroneState, DroneStatus, FleetStateManager, GPSPosition
from drone_patrol.patrol_routes import (
    generate_coverage_sweep_route,
    generate_grid_survey_route,
    generate_perimeter_patrol_route,
)
from drone_patrol.telemetry_receiver import DroneTelemetryReceiver

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "drone_patrol_app" / "static"
DASHBOARD_FILENAME = "dashboard.html"
DEFAULT_ROUTE_ALTITUDE_M = 60.0
DEFAULT_COVERAGE_SWATH_WIDTH_M = 30.0
DEFAULT_COVERAGE_OVERLAP = 0.2
DEFAULT_GRID_SPACING_M = 20.0
MAX_EVENT_LOG_ENTRIES = 30
DEFAULT_MAP_CENTER = {
    "latitude": 30.62798,
    "longitude": -96.33441,
    "zoom": 15,
}
DEFAULT_CONNECTION_PANEL = {
    "drone_id": None,
    "connection_string": "",
    "status": "disconnected",
    "message": "No MAVLink telemetry link active",
    "requested_at": None,
}


@dataclass(slots=True)
class TelemetryLink:
    drone_id: str
    connection_string: str
    requested_at: datetime
    status: str = "connecting"
    message: str = "Awaiting first telemetry frame"

fleet_manager = FleetStateManager()
state_lock = Lock()
telemetry_receivers: dict[str, DroneTelemetryReceiver] = {}
telemetry_links: dict[str, TelemetryLink] = {}
telemetry_receiver_factory = DroneTelemetryReceiver
next_drone_number = 1
server_state: dict[str, Any] = {
    "geofence": [],
    "dock_position": None,
    "patrol_route": [],
    "route_mode": "perimeter",
    "map_center": dict(DEFAULT_MAP_CENTER),
    "event_log": [],
}


def utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def require_server_dependencies() -> None:
    if Flask is None or SocketIO is None:
        raise RuntimeError(
            "Flask and Flask-SocketIO are required to run dashboard_server.py. "
            "Install them with `pip install flask flask-socketio`."
        )


def append_event(text: str, *, level: str = "info", meta: str | None = None) -> None:
    entry = {
        "timestamp": utcnow_iso(),
        "text": text,
        "level": level,
        "meta": meta or "",
    }
    with state_lock:
        server_state["event_log"].insert(0, entry)
        del server_state["event_log"][MAX_EVENT_LOG_ENTRIES:]


def seed_initial_state() -> None:
    if not server_state["event_log"]:
        append_event("System online - patrol server ready", level="info", meta="Choose a real site and draw a geofence")
        append_event("Map center not set", level="alert", meta="Use map controls to choose the patrol area")
        append_event("Awaiting live drone telemetry", level="info", meta="Fleet panel will populate as receivers connect")


def serialize_drone(drone: DroneState) -> dict[str, Any]:
    payload = drone.model_dump(mode="json")
    payload["status"] = drone.status.value
    return payload


def serialize_telemetry_link(link: TelemetryLink) -> dict[str, Any]:
    return {
        "drone_id": link.drone_id,
        "connection_string": link.connection_string,
        "status": link.status,
        "message": link.message,
        "requested_at": link.requested_at.isoformat(),
    }


def generate_drone_id() -> str:
    global next_drone_number

    with state_lock:
        while True:
            candidate = f"drone-{next_drone_number:02d}"
            next_drone_number += 1
            if candidate not in telemetry_links:
                return candidate


def find_existing_link(connection_string: str) -> TelemetryLink | None:
    with state_lock:
        for link in telemetry_links.values():
            if link.connection_string == connection_string and link.status in {"connecting", "connected"}:
                return TelemetryLink(
                    drone_id=link.drone_id,
                    connection_string=link.connection_string,
                    requested_at=link.requested_at,
                    status=link.status,
                    message=link.message,
                )
    return None


def refresh_telemetry_links(drones: list[DroneState]) -> list[dict[str, Any]]:
    drone_lookup = {drone.drone_id: drone for drone in drones}
    with state_lock:
        ordered_links = sorted(telemetry_links.values(), key=lambda link: link.requested_at)
        for link in ordered_links:
            drone = drone_lookup.get(link.drone_id)
            if drone is None:
                link.status = "connecting"
                link.message = "Awaiting first telemetry frame"
                continue
            if drone.status == DroneStatus.FAILED:
                link.status = "disconnected"
                link.message = drone.failure_reason or "Telemetry link failed"
                continue
            if drone.updated_at > link.requested_at or drone.position is not None or drone.flight_mode not in {None, "CONNECTING"}:
                link.status = "connected"
                link.message = drone.flight_mode or "Telemetry flowing"
            else:
                link.status = "connecting"
                link.message = "Awaiting first telemetry frame"
        return [serialize_telemetry_link(link) for link in ordered_links]


def build_payload() -> dict[str, Any]:
    drones = sorted(fleet_manager.snapshot(), key=lambda drone: drone.drone_id)
    telemetry_link_payload = refresh_telemetry_links(drones)
    with state_lock:
        geofence = [point.model_dump(mode="json") for point in server_state["geofence"]]
        patrol_route = [dict(item) for item in server_state["patrol_route"]]
        dock_position = server_state["dock_position"].model_dump(mode="json") if server_state["dock_position"] is not None else None
        event_log = [dict(entry) for entry in server_state["event_log"]]
        route_mode = server_state["route_mode"]
        map_center = dict(server_state["map_center"])

    drone_payload = [serialize_drone(drone) for drone in drones]
    active_drones = sum(1 for drone in drones if drone.status == DroneStatus.FLYING)
    average_battery = (sum(drone.battery_level for drone in drones) / len(drones)) if drones else 0.0
    coverage_percent = max(0, min(100, round(active_drones * 32 + average_battery * 0.36)))
    last_alert = next((entry for entry in event_log if entry["level"] in {"alert", "critical"}), event_log[0] if event_log else None)

    return {
        "generated_at": utcnow_iso(),
        "route_mode": route_mode,
        "map_center": map_center,
        "summary": {
            "total_drones": len(drones),
            "active_drones": active_drones,
            "coverage_percent": coverage_percent,
            "last_alert": last_alert,
        },
        "drones": drone_payload,
        "geofence": geofence,
        "patrol_route": patrol_route,
        "dock_position": dock_position,
        "telemetry_links": telemetry_link_payload,
        "connection_panel": telemetry_link_payload[-1] if telemetry_link_payload else dict(DEFAULT_CONNECTION_PANEL),
        "event_log": event_log,
    }


def build_signature() -> str:
    payload = build_payload()
    payload.pop("generated_at", None)
    return json.dumps(payload, sort_keys=True)


def diff_and_log(previous: dict[str, dict[str, Any]], current: dict[str, dict[str, Any]]) -> None:
    for drone_id, drone in current.items():
        prior = previous.get(drone_id)
        if prior is None:
            append_event(f"Drone {drone_id} registered", level="info", meta=f"Status {drone['status'].upper()}")
            continue

        if prior["status"] != drone["status"]:
            level = "critical" if drone["status"] == DroneStatus.FAILED.value else "alert"
            append_event(
                f"Drone {drone_id} status changed to {drone['status'].upper()}",
                level=level,
                meta=drone.get("flight_mode") or "",
            )
        elif prior["battery_level"] > 25 >= drone["battery_level"]:
            append_event(
                f"Drone {drone_id} low battery - {drone['battery_level']:.0f}%",
                level="alert",
                meta="Recall threshold reached",
            )


def parse_coordinate(point: Any) -> GPSPosition:
    if isinstance(point, GPSPosition):
        return point
    if isinstance(point, dict):
        return GPSPosition(
            latitude=point["latitude"],
            longitude=point["longitude"],
            altitude_m=point.get("altitude_m"),
        )
    if isinstance(point, (list, tuple)) and len(point) in {2, 3}:
        return GPSPosition(
            latitude=point[0],
            longitude=point[1],
            altitude_m=point[2] if len(point) == 3 else None,
        )
    raise ValueError("Each coordinate must be a dict with latitude/longitude or a 2-3 item list/tuple.")


def generate_route_for_mode(
    route_mode: str,
    points: list[GPSPosition],
    *,
    altitude_m: float,
    clockwise: bool,
    include_return_to_start: bool,
    swath_width_m: float,
    overlap_percentage: float,
    grid_spacing_m: float,
) -> list[dict[str, int | float]]:
    if route_mode == "perimeter":
        return generate_perimeter_patrol_route(
            points,
            altitude_m=altitude_m,
            clockwise=clockwise,
            include_return_to_start=include_return_to_start,
        )
    if route_mode == "coverage":
        return generate_coverage_sweep_route(
            points,
            altitude_m=altitude_m,
            swath_width_m=swath_width_m,
            overlap_percentage=overlap_percentage,
        )
    if route_mode == "grid":
        return generate_grid_survey_route(
            points,
            altitude_m=altitude_m,
            grid_spacing_m=grid_spacing_m,
        )
    raise ValueError("route_mode must be one of: perimeter, coverage, grid.")


def create_app() -> tuple[Any, Any]:
    require_server_dependencies()
    seed_initial_state()

    app = Flask(__name__, static_folder=str(STATIC_DIR), static_url_path="/static")
    app.config["SECRET_KEY"] = "drone-patrol-local-dev"
    socketio = SocketIO(app, cors_allowed_origins="*", path="ws", async_mode="threading")

    @app.get("/")
    def dashboard() -> Any:
        return send_from_directory(STATIC_DIR, DASHBOARD_FILENAME)

    @app.get("/api/fleet")
    def api_fleet() -> Any:
        return jsonify(build_payload())

    @app.post("/api/map-center")
    def api_map_center() -> Any:
        payload = request.get_json(silent=True) or {}
        try:
            latitude = float(payload["latitude"])
            longitude = float(payload["longitude"])
            zoom = int(payload.get("zoom", 16))
            center = GPSPosition(latitude=latitude, longitude=longitude)
        except (KeyError, TypeError, ValueError) as exc:
            return jsonify({"error": f"Invalid map center payload: {exc}"}), 400

        with state_lock:
            server_state["map_center"] = {
                "latitude": center.latitude,
                "longitude": center.longitude,
                "zoom": max(1, min(20, zoom)),
            }
            if server_state["dock_position"] is None:
                server_state["dock_position"] = GPSPosition(
                    latitude=center.latitude,
                    longitude=center.longitude,
                    altitude_m=0.0,
                )

        append_event("Map center updated", level="info", meta=f"{center.latitude:.5f}, {center.longitude:.5f}")
        return jsonify(build_payload())

    @app.post("/api/geofence")
    def api_geofence() -> Any:
        payload = request.get_json(silent=True)
        if isinstance(payload, list):
            coordinates = payload
            altitude_m = DEFAULT_ROUTE_ALTITUDE_M
            clockwise = True
            include_return_to_start = True
            route_mode = "perimeter"
            swath_width_m = DEFAULT_COVERAGE_SWATH_WIDTH_M
            overlap_percentage = DEFAULT_COVERAGE_OVERLAP
            grid_spacing_m = DEFAULT_GRID_SPACING_M
        elif isinstance(payload, dict):
            coordinates = payload.get("coordinates") or payload.get("geofence") or []
            altitude_m = float(payload.get("altitude_m", DEFAULT_ROUTE_ALTITUDE_M))
            clockwise = bool(payload.get("clockwise", True))
            include_return_to_start = bool(payload.get("include_return_to_start", True))
            route_mode = str(payload.get("route_mode", "perimeter")).lower()
            swath_width_m = float(payload.get("swath_width_m", DEFAULT_COVERAGE_SWATH_WIDTH_M))
            overlap_percentage = float(payload.get("overlap_percentage", DEFAULT_COVERAGE_OVERLAP))
            grid_spacing_m = float(payload.get("grid_spacing_m", DEFAULT_GRID_SPACING_M))
        else:
            return jsonify({"error": "Expected a JSON list of coordinates or an object with `coordinates`."}), 400

        try:
            points = [parse_coordinate(point) for point in coordinates]
            mission = generate_route_for_mode(
                route_mode,
                points,
                altitude_m=altitude_m,
                clockwise=clockwise,
                include_return_to_start=include_return_to_start,
                swath_width_m=swath_width_m,
                overlap_percentage=overlap_percentage,
                grid_spacing_m=grid_spacing_m,
            )
        except (KeyError, TypeError, ValueError) as exc:
            return jsonify({"error": str(exc)}), 400

        with state_lock:
            server_state["geofence"] = [point.model_copy(deep=True) for point in points]
            server_state["patrol_route"] = mission
            server_state["route_mode"] = route_mode
            centroid_lat = sum(point.latitude for point in points) / len(points)
            centroid_lon = sum(point.longitude for point in points) / len(points)
            server_state["map_center"] = {
                "latitude": centroid_lat,
                "longitude": centroid_lon,
                "zoom": 17,
            }
            server_state["dock_position"] = GPSPosition(latitude=points[0].latitude, longitude=points[0].longitude, altitude_m=0.0)

        append_event(
            "Geofence updated",
            level="info",
            meta=f"{len(points)} coordinates applied | route {route_mode.upper()}",
        )
        return jsonify(build_payload())

    @app.post("/api/connect")
    def api_connect() -> Any:
        payload = request.get_json(silent=True) or {}
        connection_string = str(payload.get("connection_string", "")).strip()
        if not connection_string:
            return jsonify({"error": "connection_string is required."}), 400

        existing_link = find_existing_link(connection_string)
        if existing_link is not None:
            append_event(
                f"Telemetry link already active for {existing_link.drone_id}",
                level="info",
                meta=existing_link.connection_string,
            )
            return jsonify(build_payload())

        drone_id = generate_drone_id()
        requested_at = datetime.now(timezone.utc)
        fleet_manager.upsert_drone(
            drone_id,
            battery_level=0.0,
            status=DroneStatus.STANDBY,
            flight_mode="CONNECTING",
            updated_at=requested_at,
            failure_reason="Awaiting telemetry link",
        )
        receiver = telemetry_receiver_factory(
            drone_id=drone_id,
            connection_string=connection_string,
            fleet_manager=fleet_manager,
        )

        with state_lock:
            telemetry_receivers[drone_id] = receiver
            telemetry_links[drone_id] = TelemetryLink(
                drone_id=drone_id,
                connection_string=connection_string,
                requested_at=requested_at,
            )

        receiver.start()
        append_event(
            f"Telemetry link requested for {drone_id}",
            level="info",
            meta=connection_string,
        )
        return jsonify(build_payload()), 202

    @socketio.on("connect")
    def on_connect() -> None:
        socketio.emit("fleet_state", build_payload())

    def fleet_push_loop() -> None:
        previous_drones = {
            drone["drone_id"]: drone
            for drone in [serialize_drone(item) for item in sorted(fleet_manager.snapshot(), key=lambda x: x.drone_id)]
        }
        emitted_signature = build_signature()

        while True:
            socketio.sleep(1)
            current_drones = {
                drone["drone_id"]: drone
                for drone in [serialize_drone(item) for item in sorted(fleet_manager.snapshot(), key=lambda x: x.drone_id)]
            }
            if current_drones != previous_drones:
                diff_and_log(previous_drones, current_drones)
                previous_drones = current_drones

            current_signature = build_signature()
            if current_signature != emitted_signature:
                socketio.emit("fleet_state", build_payload())
                emitted_signature = current_signature

    socketio.start_background_task(fleet_push_loop)
    return app, socketio


if Flask is not None and SocketIO is not None:
    app, socketio = create_app()
else:  # pragma: no cover
    app = None
    socketio = None


if __name__ == "__main__":
    require_server_dependencies()
    assert socketio is not None
    socketio.run(
        app,
        host="127.0.0.1",
        port=8000,
        debug=True,
        use_reloader=False,
        allow_unsafe_werkzeug=True,
    )
