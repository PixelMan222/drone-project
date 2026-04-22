from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager

import dashboard_server
from drone_patrol.fleet_state import DroneStatus


@contextmanager
def reset_dashboard_state() -> Iterator[None]:
    original_factory = dashboard_server.telemetry_receiver_factory
    original_mission_uploader = dashboard_server.mission_uploader_callable
    with dashboard_server.state_lock:
        dashboard_server.server_state["geofence"] = []
        dashboard_server.server_state["dock_position"] = None
        dashboard_server.server_state["patrol_route"] = []
        dashboard_server.server_state["route_mode"] = "perimeter"
        dashboard_server.server_state["map_center"] = dict(dashboard_server.DEFAULT_MAP_CENTER)
        dashboard_server.server_state["event_log"] = []
        dashboard_server.telemetry_receivers.clear()
        dashboard_server.telemetry_links.clear()
        dashboard_server.next_drone_number = 1
    with dashboard_server.fleet_manager._lock:  # noqa: SLF001 - test reset helper
        dashboard_server.fleet_manager._drones.clear()  # noqa: SLF001 - test reset helper
    dashboard_server.seed_initial_state()
    try:
        yield
    finally:
        dashboard_server.telemetry_receiver_factory = original_factory
        dashboard_server.mission_uploader_callable = original_mission_uploader
        with dashboard_server.state_lock:
            dashboard_server.server_state["geofence"] = []
            dashboard_server.server_state["dock_position"] = None
            dashboard_server.server_state["patrol_route"] = []
            dashboard_server.server_state["route_mode"] = "perimeter"
            dashboard_server.server_state["map_center"] = dict(dashboard_server.DEFAULT_MAP_CENTER)
            dashboard_server.server_state["event_log"] = []
            dashboard_server.telemetry_receivers.clear()
            dashboard_server.telemetry_links.clear()
            dashboard_server.next_drone_number = 1
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


def test_api_connect_registers_receiver_and_returns_connecting_panel() -> None:
    with reset_dashboard_state():
        created_receivers = []

        class FakeReceiver:
            def __init__(self, **kwargs) -> None:
                self.kwargs = kwargs
                self.started = False
                created_receivers.append(self)

            def start(self) -> None:
                self.started = True

        dashboard_server.telemetry_receiver_factory = FakeReceiver
        client = dashboard_server.app.test_client()

        response = client.post(
            "/api/connect",
            json={"connection_string": "tcp:127.0.0.1:5760"},
        )

        assert response.status_code == 202
        assert len(created_receivers) == 1
        assert created_receivers[0].started is True
        assert created_receivers[0].kwargs["connection_string"] == "tcp:127.0.0.1:5760"
        assert created_receivers[0].kwargs["drone_id"] == "drone-01"

        payload = response.get_json()
        assert payload["connection_panel"]["status"] == "connecting"
        assert payload["connection_panel"]["connection_string"] == "tcp:127.0.0.1:5760"
        assert payload["connection_panel"]["drone_id"] == "drone-01"
        assert payload["drones"][0]["drone_id"] == "drone-01"
        assert payload["drones"][0]["flight_mode"] == "CONNECTING"


def test_api_connect_reflects_connected_status_once_telemetry_updates() -> None:
    with reset_dashboard_state():
        class FakeReceiver:
            def __init__(self, **kwargs) -> None:
                self.kwargs = kwargs

            def start(self) -> None:
                self.kwargs["fleet_manager"].update_drone(
                    self.kwargs["drone_id"],
                    battery_level=87.0,
                    status=DroneStatus.FLYING,
                    flight_mode="AUTO",
                    armed=True,
                )

        dashboard_server.telemetry_receiver_factory = FakeReceiver
        client = dashboard_server.app.test_client()

        response = client.post(
            "/api/connect",
            json={"connection_string": "udp:127.0.0.1:14550"},
        )

        assert response.status_code == 202
        payload = response.get_json()
        assert payload["connection_panel"]["status"] == "connected"
        assert payload["connection_panel"]["message"] == "AUTO"
        assert payload["drones"][0]["status"] == "flying"


def test_api_connect_requires_connection_string() -> None:
    with reset_dashboard_state():
        client = dashboard_server.app.test_client()

        response = client.post("/api/connect", json={"connection_string": "   "})

        assert response.status_code == 400
        payload = response.get_json()
        assert "connection_string" in payload["error"]


def test_api_upload_mission_uses_active_route_and_connected_drone() -> None:
    with reset_dashboard_state():
        uploaded_calls = []

        def fake_mission_uploader(*, connection_string, mission_waypoints):
            uploaded_calls.append(
                {
                    "connection_string": connection_string,
                    "mission_waypoints": mission_waypoints,
                }
            )
            return {
                "connection_string": connection_string,
                "uploaded_waypoints": len(mission_waypoints),
                "mode": "AUTO",
            }

        dashboard_server.mission_uploader_callable = fake_mission_uploader
        with dashboard_server.state_lock:
            dashboard_server.server_state["patrol_route"] = [
                {
                    "seq": 0,
                    "frame": 3,
                    "command": 16,
                    "current": 1,
                    "autocontinue": 1,
                    "param1": 0.0,
                    "param2": 2.0,
                    "param3": 0.0,
                    "param4": 0.0,
                    "x_lat": 30.6275,
                    "y_lon": -96.3351,
                    "z_alt": 60.0,
                },
                {
                    "seq": 1,
                    "frame": 3,
                    "command": 16,
                    "current": 0,
                    "autocontinue": 1,
                    "param1": 0.0,
                    "param2": 2.0,
                    "param3": 0.0,
                    "param4": 0.0,
                    "x_lat": 30.6284,
                    "y_lon": -96.3339,
                    "z_alt": 60.0,
                },
            ]
            dashboard_server.telemetry_links["drone-01"] = dashboard_server.TelemetryLink(
                drone_id="drone-01",
                connection_string="tcp:127.0.0.1:5760",
                requested_at=dashboard_server.datetime.now(dashboard_server.timezone.utc),
                status="connected",
                message="AUTO",
            )

        client = dashboard_server.app.test_client()
        response = client.post("/api/upload-mission", json={"drone_id": "drone-01"})

        assert response.status_code == 200
        assert uploaded_calls[0]["connection_string"] == "tcp:127.0.0.1:5760"
        assert len(uploaded_calls[0]["mission_waypoints"]) == 2
        payload = response.get_json()
        assert payload["upload_result"]["uploaded_waypoints"] == 2
        assert any(entry["text"] == "Mission uploaded to drone-01" for entry in payload["payload"]["event_log"])


def test_api_upload_mission_requires_active_route() -> None:
    with reset_dashboard_state():
        with dashboard_server.state_lock:
            dashboard_server.telemetry_links["drone-01"] = dashboard_server.TelemetryLink(
                drone_id="drone-01",
                connection_string="udp:127.0.0.1:14550",
                requested_at=dashboard_server.datetime.now(dashboard_server.timezone.utc),
                status="connected",
                message="AUTO",
            )
        client = dashboard_server.app.test_client()

        response = client.post("/api/upload-mission", json={"drone_id": "drone-01"})

        assert response.status_code == 400
        payload = response.get_json()
        assert "No active patrol route" in payload["error"]
