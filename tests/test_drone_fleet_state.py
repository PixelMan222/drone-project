from drone_patrol.fleet_state import (
    DroneCommandType,
    DroneState,
    DroneStatus,
    FleetPolicy,
    FleetStateManager,
    GPSPosition,
)


def test_rotation_recalls_low_battery_drone_and_launches_best_replacement() -> None:
    manager = FleetStateManager(
        drones=[
            DroneState(
                drone_id="alpha",
                battery_level=22.0,
                status=DroneStatus.FLYING,
                position=GPSPosition(latitude=41.0, longitude=-87.0, altitude_m=110.0),
            ),
            DroneState(
                drone_id="bravo",
                battery_level=96.0,
                status=DroneStatus.STANDBY,
                position=GPSPosition(latitude=41.1, longitude=-87.1, altitude_m=0.0),
            ),
            DroneState(
                drone_id="charlie",
                battery_level=91.0,
                status=DroneStatus.STANDBY,
                position=GPSPosition(latitude=41.2, longitude=-87.2, altitude_m=0.0),
            ),
        ],
        policy=FleetPolicy(recall_battery_threshold=25.0, minimum_launch_battery=90.0, target_airborne_drones=1),
    )

    decision = manager.evaluate_rotation()

    assert [command.command for command in decision.commands] == [
        DroneCommandType.RECALL_TO_CHARGER,
        DroneCommandType.LAUNCH_TO_PATROL,
    ]
    assert decision.commands[0].drone_id == "alpha"
    assert decision.commands[1].drone_id == "bravo"
    assert decision.active_drone_ids == ["bravo"]
    assert decision.unmet_airborne_slots == 0


def test_rotation_backfills_when_airborne_count_is_below_target() -> None:
    manager = FleetStateManager(
        drones=[
            DroneState(
                drone_id="alpha",
                battery_level=82.0,
                status=DroneStatus.FLYING,
                position=GPSPosition(latitude=41.0, longitude=-87.0),
            ),
            DroneState(
                drone_id="bravo",
                battery_level=95.0,
                status=DroneStatus.STANDBY,
                position=GPSPosition(latitude=41.1, longitude=-87.1),
            ),
        ],
        policy=FleetPolicy(recall_battery_threshold=25.0, minimum_launch_battery=90.0, target_airborne_drones=2),
    )

    decision = manager.evaluate_rotation()

    assert len(decision.commands) == 1
    assert decision.commands[0].command == DroneCommandType.LAUNCH_TO_PATROL
    assert decision.commands[0].drone_id == "bravo"
    assert decision.active_drone_ids == ["alpha", "bravo"]
    assert decision.unmet_airborne_slots == 0


def test_rotation_does_not_launch_standby_drone_below_launch_threshold() -> None:
    manager = FleetStateManager(
        drones=[
            DroneState(
                drone_id="alpha",
                battery_level=20.0,
                status=DroneStatus.FLYING,
                position=GPSPosition(latitude=41.0, longitude=-87.0),
            ),
            DroneState(
                drone_id="bravo",
                battery_level=80.0,
                status=DroneStatus.STANDBY,
                position=GPSPosition(latitude=41.1, longitude=-87.1),
            ),
        ],
        policy=FleetPolicy(recall_battery_threshold=25.0, minimum_launch_battery=90.0, target_airborne_drones=1),
    )

    decision = manager.evaluate_rotation()

    assert len(decision.commands) == 1
    assert decision.commands[0].command == DroneCommandType.RECALL_TO_CHARGER
    assert decision.commands[0].drone_id == "alpha"
    assert decision.active_drone_ids == []
    assert decision.ready_standby_ids == []
    assert decision.unmet_airborne_slots == 1


def test_manager_preserves_last_known_position_and_failure_reason() -> None:
    manager = FleetStateManager()
    manager.upsert_drone(
        "alpha",
        battery_level=77.0,
        status=DroneStatus.STANDBY,
        position=GPSPosition(latitude=41.0, longitude=-87.0, altitude_m=5.0),
    )

    manager.mark_failed("alpha", failure_reason="motor controller fault")
    drone = manager.get_drone("alpha")

    assert drone.status == DroneStatus.FAILED
    assert drone.failure_reason == "motor controller fault"
    assert drone.position is not None
    assert drone.position.latitude == 41.0


def test_manager_updates_heading_mode_and_armed_state() -> None:
    manager = FleetStateManager()
    manager.upsert_drone(
        "alpha",
        battery_level=77.0,
        status=DroneStatus.STANDBY,
        position=GPSPosition(latitude=41.0, longitude=-87.0, altitude_m=5.0),
    )

    manager.update_drone(
        "alpha",
        heading_deg=182.5,
        flight_mode="AUTO",
        armed=True,
    )
    drone = manager.get_drone("alpha")

    assert drone.heading_deg == 182.5
    assert drone.flight_mode == "AUTO"
    assert drone.armed is True


def test_rotation_does_not_repeat_recall_when_drone_is_already_in_rtl() -> None:
    manager = FleetStateManager(
        drones=[
            DroneState(
                drone_id="alpha",
                battery_level=20.0,
                status=DroneStatus.FLYING,
                flight_mode="RTL",
                position=GPSPosition(latitude=41.0, longitude=-87.0),
            ),
            DroneState(
                drone_id="bravo",
                battery_level=95.0,
                status=DroneStatus.STANDBY,
                position=GPSPosition(latitude=41.1, longitude=-87.1),
            ),
        ],
        policy=FleetPolicy(recall_battery_threshold=25.0, minimum_launch_battery=90.0, target_airborne_drones=1),
    )

    decision = manager.evaluate_rotation()

    assert len(decision.commands) == 0
    assert decision.active_drone_ids == ["alpha"]
