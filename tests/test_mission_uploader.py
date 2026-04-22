from __future__ import annotations

import pytest

from drone_patrol.mission_uploader import upload_mission


class FakeCommands:
    def __init__(self) -> None:
        self.actions: list[str] = []
        self.added: list[tuple] = []

    def download(self) -> None:
        self.actions.append("download")

    def wait_ready(self) -> None:
        self.actions.append("wait_ready")

    def clear(self) -> None:
        self.actions.append("clear")

    def add(self, command: tuple) -> None:
        self.actions.append("add")
        self.added.append(command)

    def upload(self) -> None:
        self.actions.append("upload")


class FakeVehicle:
    def __init__(self) -> None:
        self.commands = FakeCommands()
        self.mode = None
        self.flush_called = False
        self.closed = False

    def flush(self) -> None:
        self.flush_called = True

    def close(self) -> None:
        self.closed = True


def test_upload_mission_clears_uploads_and_sets_auto_mode() -> None:
    vehicle = FakeVehicle()
    calls: list[tuple[str, dict]] = []

    result = upload_mission(
        connection_string="tcp:127.0.0.1:5760",
        mission_waypoints=[
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
        ],
        connect_callable=lambda connection_string, **kwargs: calls.append((connection_string, kwargs)) or vehicle,
        command_factory=lambda *args: args,
        vehicle_mode_factory=lambda mode_name: f"MODE:{mode_name}",
    )

    assert calls[0][0] == "tcp:127.0.0.1:5760"
    assert calls[0][1]["wait_ready"] is True
    assert vehicle.commands.actions == ["download", "wait_ready", "clear", "add", "add", "upload"]
    assert vehicle.mode == "MODE:AUTO"
    assert vehicle.flush_called is True
    assert vehicle.closed is True
    assert result["uploaded_waypoints"] == 2
    assert result["mode"] == "AUTO"


def test_upload_mission_requires_waypoints() -> None:
    with pytest.raises(ValueError, match="at least one waypoint"):
        upload_mission(connection_string="udp:127.0.0.1:14550", mission_waypoints=[])
