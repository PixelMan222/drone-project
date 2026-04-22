# Drone Patrol App

This folder contains the local application-facing files for the drone patrol system.

## Layout

- `static/dashboard.html`: standalone tactical dashboard UI

## Related Code

- `src/drone_patrol/`: reusable Python package for fleet state, patrol routes, and telemetry receivers
- `tests/test_drone_fleet_state.py`: fleet manager tests
- `tests/test_patrol_routes.py`: patrol route generator tests
- `tests/test_telemetry_receiver.py`: telemetry receiver tests

## Intended Next Step

If we add a local web server later, `dashboard_server.py` can live in this folder and serve `static/dashboard.html` directly.
