from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager

import dashboard_server


@contextmanager
def reset_dashboard_state() -> Iterator[None]:
    with dashboard_server.state_lock:
        dashboard_server.server_state["geofence"] = []
        dashboard_server.server_state["dock_position"] = None
        dashboard_server.server_state["patrol_route"] = []
        dashboard_server.server_state["route_mode"] = "perimeter"
        dashboard_server.server_state["map_center"] = dict(dashboard_server.DEFAULT_MAP_CENTER)
        dashboard_server.server_state["event_log"] = []
    with dashboard_server.fleet_manager._lock:  # noqa: SLF001 - test reset helper
        dashboard_server.fleet_manager._drones.clear()  # noqa: SLF001 - test reset helper
    dashboard_server.seed_initial_state()
    try:
        yield
    finally:
        with dashboard_server.state_lock:
            dashboard_server.server_state["geofence"] = []
            dashboard_server.server_state["dock_position"] = None
            dashboard_server.server_state["patrol_route"] = []
            dashboard_server.server_state["route_mode"] = "perimeter"
            dashboard_server.server_state["map_center"] = dict(dashboard_server.DEFAULT_MAP_CENTER)
            dashboard_server.server_state["event_log"] = []
        with dashboard_server.fleet_manager._lock:  # noqa: SLF001 - test reset helper
            dashboard_server.fleet_manager._drones.clear()  # noqa: SLF001 - test reset helper
        dashboard_server.seed_initial_state()


def test_api_fleet_defaults_to_college_station_map_center() -> None:
    with reset_dashboard_state():
        client = dashboard_server.app.test_client()

        response = client.get("/api/fleet")

        assert response.status_code == 200
        payload = response.get_json()
        assert payload["map_center"] == dashboard_server.DEFAULT_MAP_CENTER
        assert payload["route_mode"] == "perimeter"


def test_api_geofence_generates_selected_route_mode() -> None:
    with reset_dashboard_state():
        client = dashboard_server.app.test_client()
        coordinates = [
            {"latitude": 30.6275, "longitude": -96.3351},
            {"latitude": 30.6275, "longitude": -96.3339},
            {"latitude": 30.6284, "longitude": -96.3339},
            {"latitude": 30.6284, "longitude": -96.3351},
        ]

        response = client.post(
            "/api/geofence",
            json={
                "coordinates": coordinates,
                "route_mode": "coverage",
            },
        )

        assert response.status_code == 200
        payload = response.get_json()
        assert payload["route_mode"] == "coverage"
        assert [
            {"latitude": point["latitude"], "longitude": point["longitude"]}
            for point in payload["geofence"]
        ] == coordinates
        assert len(payload["patrol_route"]) >= 4
        assert payload["dock_position"]["latitude"] == coordinates[0]["latitude"]
        assert payload["dock_position"]["longitude"] == coordinates[0]["longitude"]


def test_api_geofence_rejects_unknown_route_mode() -> None:
    with reset_dashboard_state():
        client = dashboard_server.app.test_client()

        response = client.post(
            "/api/geofence",
            json={
                "coordinates": [
                    {"latitude": 30.6275, "longitude": -96.3351},
                    {"latitude": 30.6275, "longitude": -96.3339},
                    {"latitude": 30.6284, "longitude": -96.3339},
                ],
                "route_mode": "spiral",
            },
        )

        assert response.status_code == 400
        payload = response.get_json()
        assert "route_mode" in payload["error"]
