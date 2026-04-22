import pytest

from drone_patrol.fleet_state import GPSPosition
from drone_patrol.patrol_routes import (
    generate_coverage_sweep_route,
    generate_grid_survey_route,
    generate_perimeter_patrol_route,
)


def test_generate_perimeter_patrol_route_outputs_closed_clockwise_mission() -> None:
    geofence = [
        GPSPosition(latitude=30.0, longitude=-97.0),
        GPSPosition(latitude=30.0, longitude=-96.0),
        GPSPosition(latitude=31.0, longitude=-96.0),
        GPSPosition(latitude=31.0, longitude=-97.0),
        GPSPosition(latitude=30.0, longitude=-97.0),
    ]

    mission = generate_perimeter_patrol_route(geofence, altitude_m=60.0)

    assert [item["seq"] for item in mission] == [0, 1, 2, 3, 4]
    assert mission[0]["current"] == 1
    assert all(item["command"] == 16 for item in mission)
    assert all(item["frame"] == 3 for item in mission)
    assert mission[0]["x_lat"] == pytest.approx(30.0)
    assert mission[0]["y_lon"] == pytest.approx(-97.0)
    assert mission[1]["x_lat"] == pytest.approx(31.0)
    assert mission[1]["y_lon"] == pytest.approx(-97.0)
    assert mission[-1]["x_lat"] == pytest.approx(mission[0]["x_lat"])
    assert mission[-1]["y_lon"] == pytest.approx(mission[0]["y_lon"])


def test_generate_perimeter_patrol_route_uses_point_altitude_when_present() -> None:
    geofence = [
        (30.0, -97.0, 50.0),
        (31.0, -97.0, 55.0),
        (31.0, -96.0, 60.0),
    ]

    mission = generate_perimeter_patrol_route(
        geofence,
        altitude_m=80.0,
        include_return_to_start=False,
        clockwise=False,
    )

    assert [item["x_lat"] for item in mission] == [30.0, 31.0, 31.0]
    assert [item["y_lon"] for item in mission] == [-97.0, -96.0, -97.0]
    assert [item["z_alt"] for item in mission] == [50.0, 60.0, 55.0]
    assert [item["seq"] for item in mission] == [0, 1, 2]


def test_generate_perimeter_patrol_route_rejects_invalid_geofence() -> None:
    with pytest.raises(ValueError, match="at least three"):
        generate_perimeter_patrol_route([(30.0, -97.0), (31.0, -97.0)], altitude_m=60.0)

    with pytest.raises(ValueError, match="greater than 0"):
        generate_perimeter_patrol_route(
            [(30.0, -97.0), (31.0, -97.0), (31.0, -96.0)],
            altitude_m=0.0,
        )


def test_generate_coverage_sweep_route_returns_multiple_passes() -> None:
    geofence = [
        GPSPosition(latitude=30.0000, longitude=-97.0000),
        GPSPosition(latitude=30.0000, longitude=-96.9988),
        GPSPosition(latitude=30.0010, longitude=-96.9988),
        GPSPosition(latitude=30.0010, longitude=-97.0000),
    ]

    mission = generate_coverage_sweep_route(
        geofence,
        altitude_m=70.0,
        swath_width_m=25.0,
        overlap_percentage=0.2,
    )

    assert len(mission) >= 4
    assert mission[0]["current"] == 1
    assert all(item["command"] == 16 for item in mission)
    assert all(30.0000 <= item["x_lat"] <= 30.0010 for item in mission)
    assert all(-97.0000 <= item["y_lon"] <= -96.9988 for item in mission)


def test_generate_coverage_sweep_route_validates_overlap() -> None:
    with pytest.raises(ValueError, match="overlap_percentage"):
        generate_coverage_sweep_route(
            [(30.0, -97.0), (30.0, -96.999), (30.001, -96.999)],
            altitude_m=60.0,
            swath_width_m=20.0,
            overlap_percentage=1.0,
        )


def test_generate_grid_survey_route_generates_grid_intersections() -> None:
    geofence = [
        GPSPosition(latitude=30.0000, longitude=-97.0000),
        GPSPosition(latitude=30.0000, longitude=-96.9990),
        GPSPosition(latitude=30.0010, longitude=-96.9990),
        GPSPosition(latitude=30.0010, longitude=-97.0000),
    ]

    mission = generate_grid_survey_route(
        geofence,
        altitude_m=75.0,
        grid_spacing_m=40.0,
    )

    assert len(mission) >= 6
    assert mission[0]["current"] == 1
    assert all(item["frame"] == 3 for item in mission)
    assert all(item["z_alt"] == 75.0 for item in mission)


def test_generate_grid_survey_route_rejects_invalid_spacing() -> None:
    with pytest.raises(ValueError, match="grid_spacing_m"):
        generate_grid_survey_route(
            [(30.0, -97.0), (30.0, -96.999), (30.001, -96.999)],
            altitude_m=60.0,
            grid_spacing_m=0.0,
        )
