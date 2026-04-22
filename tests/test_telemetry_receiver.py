from __future__ import annotations

from dataclasses import dataclass
from time import monotonic, sleep

from drone_patrol.fleet_state import DroneStatus, FleetStateManager
from drone_patrol.telemetry_receiver import DroneTelemetryReceiver


@dataclass
class FakeBattery:
    level: float | None


@dataclass
class FakeMode:
    name: str


@dataclass
class FakeFrame:
    lat: float
    lon: float
    alt: float


@dataclass
class FakeLocation:
    global_relative_frame: FakeFrame


class FakeVehicle:
    def __init__(
        self,
        *,
        battery_level: float,
        lat: float,
        lon: float,
        alt: float,
        heading: float,
        mode: str,
        armed: bool,
    ) -> None:
        self.battery = FakeBattery(level=battery_level)
        self.location = FakeLocation(global_relative_frame=FakeFrame(lat=lat, lon=lon, alt=alt))
        self.heading = heading
        self.mode = FakeMode(name=mode)
        self.armed = armed
        self.closed = False

    def close(self) -> None:
        self.closed = True


def wait_for(predicate, timeout: float = 1.0) -> bool:
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        try:
            if predicate():
                return True
        except KeyError:
            pass
        sleep(0.01)
    try:
        if predicate():
            return True
    except KeyError:
        pass
    return False


def test_receiver_updates_fleet_manager_from_vehicle_telemetry() -> None:
    manager = FleetStateManager()
    vehicle = FakeVehicle(
        battery_level=83.0,
        lat=35.2219,
        lon=-101.7762,
        alt=52.0,
        heading=91.0,
        mode="AUTO",
        armed=True,
    )

    receiver = DroneTelemetryReceiver(
        drone_id="alpha",
        connection_string="tcp:127.0.0.1:5760",
        fleet_manager=manager,
        poll_interval_s=0.02,
        connect_callable=lambda *_args, **_kwargs: vehicle,
    )

    receiver.start()
    assert wait_for(lambda: manager.get_drone("alpha").armed is True)
    receiver.stop()
    receiver.join(timeout=1.0)

    drone = manager.get_drone("alpha")
    assert drone.status == DroneStatus.FLYING
    assert drone.battery_level == 83.0
    assert drone.position is not None
    assert drone.position.latitude == 35.2219
    assert drone.heading_deg == 91.0
    assert drone.flight_mode == "AUTO"
    assert vehicle.closed is True


def test_multiple_receivers_can_run_simultaneously() -> None:
    manager = FleetStateManager()
    alpha_vehicle = FakeVehicle(
        battery_level=78.0,
        lat=35.22,
        lon=-101.77,
        alt=48.0,
        heading=45.0,
        mode="AUTO",
        armed=True,
    )
    bravo_vehicle = FakeVehicle(
        battery_level=66.0,
        lat=35.225,
        lon=-101.779,
        alt=0.0,
        heading=180.0,
        mode="HOLD",
        armed=False,
    )

    alpha = DroneTelemetryReceiver(
        drone_id="alpha",
        connection_string="udp:127.0.0.1:14550",
        fleet_manager=manager,
        poll_interval_s=0.02,
        connect_callable=lambda *_args, **_kwargs: alpha_vehicle,
    )
    bravo = DroneTelemetryReceiver(
        drone_id="bravo",
        connection_string="tcp:127.0.0.1:5760",
        fleet_manager=manager,
        poll_interval_s=0.02,
        connect_callable=lambda *_args, **_kwargs: bravo_vehicle,
    )

    alpha.start()
    bravo.start()
    assert wait_for(lambda: len(manager.list_drones()) == 2)
    alpha.stop()
    bravo.stop()
    alpha.join(timeout=1.0)
    bravo.join(timeout=1.0)

    alpha_state = manager.get_drone("alpha")
    bravo_state = manager.get_drone("bravo")
    assert alpha_state.status == DroneStatus.FLYING
    assert bravo_state.status == DroneStatus.STANDBY
    assert bravo_state.flight_mode == "HOLD"


def test_receiver_marks_drone_failed_when_connection_errors() -> None:
    manager = FleetStateManager()

    receiver = DroneTelemetryReceiver(
        drone_id="charlie",
        connection_string="serial:/dev/ttyUSB0",
        fleet_manager=manager,
        poll_interval_s=0.02,
        connect_callable=lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("link timeout")),
    )

    receiver.start()
    assert wait_for(lambda: manager.get_drone("charlie").status == DroneStatus.FAILED)
    receiver.join(timeout=1.0)

    drone = manager.get_drone("charlie")
    assert drone.failure_reason == "link timeout"
