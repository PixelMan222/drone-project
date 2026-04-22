from __future__ import annotations

import pytest

from drone_patrol.mission_uploader import MissionUploadError, upload_mission


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
        self.sent_mavlink: list[tuple] = []
        self.message_factory = type(
            "FakeMessageFactory",
            (),
            {
                "mission_item_int_encode": staticmethod(lambda *args: args),
                "param_set_encode": staticmethod(lambda *args: args),
            },
        )()
        self.parameters: dict[str, int] = {}
        self.mode_history: list[str] = []
        self._mode = None
        self._armed = False
        self.is_armable = True
        self.system_status = type("FakeSystemStatus", (), {"state": "STANDBY"})()
        self.gps_0 = type("FakeGps", (), {"fix_type": 0, "satellites_visible": 0})()
        self.last_heartbeat = 0.2
        self.flush_called = False
        self.closed = False

    @property
    def mode(self):  # type: ignore[no-untyped-def]
        return self._mode

    @mode.setter
    def mode(self, value) -> None:  # type: ignore[no-untyped-def]
        self._mode = value
        self.mode_history.append(value)

    @property
    def armed(self) -> bool:
        return self._armed

    @armed.setter
    def armed(self, value: bool) -> None:
        self._armed = value

    def flush(self) -> None:
        self.flush_called = True

    def send_mavlink(self, command: tuple) -> None:
        self.sent_mavlink.append(command)

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
        vehicle_mode_factory=lambda mode_name: mode_name,
    )

    assert calls[0][0] == "tcp:127.0.0.1:5760"
    assert calls[0][1]["wait_ready"] is True
    assert vehicle.commands.actions == ["download", "wait_ready", "clear", "add", "add", "upload"]
    assert vehicle.parameters["ARMING_CHECK"] == 0
    assert vehicle.mode_history == ["GUIDED", "AUTO"]
    assert vehicle.mode == "AUTO"
    assert vehicle.armed is True
    assert vehicle.flush_called is True
    assert vehicle.closed is True
    assert result["uploaded_waypoints"] == 2
    assert result["mode"] == "AUTO"
    assert result["armed"] is True
    assert vehicle.commands.added[0][11] == 306275000
    assert vehicle.commands.added[0][12] == -963351000


def test_upload_mission_requires_waypoints() -> None:
    with pytest.raises(ValueError, match="at least one waypoint"):
        upload_mission(connection_string="udp:127.0.0.1:14550", mission_waypoints=[])


def test_upload_mission_logs_traceback_and_step_on_failure(capsys: pytest.CaptureFixture[str]) -> None:
    vehicle = FakeVehicle()

    def fail_clear() -> None:
        raise RuntimeError("clear link timeout")

    vehicle.commands.clear = fail_clear  # type: ignore[method-assign]

    with pytest.raises(MissionUploadError, match="clearing mission"):
        upload_mission(
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
                }
            ],
            connect_callable=lambda *_args, **_kwargs: vehicle,
            vehicle_mode_factory=lambda mode_name: mode_name,
        )

    captured = capsys.readouterr()
    assert "Mission upload failed during clearing mission" in captured.out
    assert "RuntimeError: clear link timeout" in captured.err


def test_upload_mission_requires_auto_mode_confirmation() -> None:
    vehicle = FakeVehicle()

    with pytest.raises(MissionUploadError, match="setting AUTO mode"):
        upload_mission(
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
                }
            ],
            connect_callable=lambda *_args, **_kwargs: vehicle,
            vehicle_mode_factory=lambda mode_name: "GUIDED" if mode_name == "GUIDED" else "MODE:GUIDED",
            mode_change_timeout_s=0.01,
        )


def test_upload_mission_times_out_when_vehicle_never_arms(capsys: pytest.CaptureFixture[str]) -> None:
    class NeverArmsVehicle(FakeVehicle):
        @FakeVehicle.armed.setter
        def armed(self, value: bool) -> None:
            self._armed = False

    vehicle = NeverArmsVehicle()
    vehicle.is_armable = False
    vehicle.system_status.state = "CRITICAL"
    vehicle.gps_0.fix_type = 1
    vehicle.gps_0.satellites_visible = 3

    with pytest.raises(MissionUploadError, match="arming vehicle"):
        upload_mission(
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
                }
            ],
            connect_callable=lambda *_args, **_kwargs: vehicle,
            vehicle_mode_factory=lambda mode_name: mode_name,
            arm_timeout_s=0.01,
        )

    captured = capsys.readouterr()
    assert "Mission upload failed during arming vehicle" in captured.out
    assert "is_armable=False" in captured.err
    assert "system_status=CRITICAL" in captured.err
