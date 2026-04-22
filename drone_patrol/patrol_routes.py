from __future__ import annotations

from collections.abc import Iterable, Sequence
from math import atan2, ceil, cos, isclose, pi, radians, sin

from pydantic import BaseModel, Field

from .fleet_state import GPSPosition

MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_DO_JUMP = 177
EARTH_RADIUS_M = 6_371_000.0
EPSILON = 1e-9


class MAVLinkMissionWaypoint(BaseModel):
    seq: int = Field(ge=0)
    frame: int = MAV_FRAME_GLOBAL_RELATIVE_ALT
    command: int = MAV_CMD_NAV_WAYPOINT
    current: int = Field(default=0, ge=0, le=1)
    autocontinue: int = Field(default=1, ge=0, le=1)
    param1: float = 0.0
    param2: float = 2.0
    param3: float = 0.0
    param4: float = 0.0
    x_lat: float = Field(ge=-90.0, le=90.0)
    y_lon: float = Field(ge=-180.0, le=180.0)
    z_alt: float

    def to_mission_item(self) -> dict[str, int | float]:
        return self.model_dump()


def generate_perimeter_patrol_route(
    geofence: Sequence[GPSPosition | tuple[float, float] | tuple[float, float, float]],
    *,
    altitude_m: float,
    clockwise: bool = True,
    include_return_to_start: bool = True,
    hold_time_s: float = 0.0,
    acceptance_radius_m: float = 2.0,
    pass_radius_m: float = 0.0,
    yaw_deg: float = 0.0,
) -> list[dict[str, int | float]]:
    """Generate a perimeter patrol mission around the boundary vertices."""

    _validate_altitude(altitude_m)
    boundary = _normalize_boundary(geofence)
    oriented_boundary = _ensure_orientation(boundary, clockwise=clockwise)

    route_points = list(oriented_boundary)
    if include_return_to_start:
        route_points.append(oriented_boundary[0])

    return _build_mission_items(
        route_points,
        altitude_m=altitude_m,
        hold_time_s=hold_time_s,
        acceptance_radius_m=acceptance_radius_m,
        pass_radius_m=pass_radius_m,
        yaw_deg=yaw_deg,
    )


def generate_coverage_sweep_route(
    geofence: Sequence[GPSPosition | tuple[float, float] | tuple[float, float, float]],
    *,
    altitude_m: float,
    swath_width_m: float = 30.0,
    overlap_percentage: float = 0.2,
    hold_time_s: float = 0.0,
    acceptance_radius_m: float = 2.0,
    pass_radius_m: float = 0.0,
    yaw_deg: float = 0.0,
) -> list[dict[str, int | float]]:
    """
    Generate a lawnmower/boustrophedon sweep over the geofence interior.

    The sweep axis is aligned to the polygon's principal axis so the passes run along
    the longest dimension and the total reposition distance is reduced.
    """

    _validate_altitude(altitude_m)
    if swath_width_m <= 0:
        raise ValueError("swath_width_m must be greater than 0.")
    if not 0.0 <= overlap_percentage < 1.0:
        raise ValueError("overlap_percentage must be between 0.0 and 1.0.")

    boundary = _normalize_boundary(geofence)
    reference = _projection_reference(boundary)
    local_boundary = [_to_local_xy(point, reference) for point in boundary]
    sweep_angle = _principal_axis_angle(local_boundary)
    rotated_boundary = [_rotate_xy(point, -sweep_angle) for point in local_boundary]

    effective_spacing_m = swath_width_m * (1.0 - overlap_percentage)
    sweep_lines = _coverage_scan_lines(rotated_boundary, effective_spacing_m)

    route_local: list[tuple[float, float]] = []
    reverse_row = False
    for y in sweep_lines:
        intervals = _scanline_intervals(rotated_boundary, y)
        if not intervals:
            continue

        ordered_intervals = list(reversed(intervals)) if reverse_row else intervals
        for x_start, x_end in ordered_intervals:
            row_points = [(x_start, y), (x_end, y)]
            if reverse_row:
                row_points.reverse()
            route_local.extend(_rotate_xy(point, sweep_angle) for point in row_points)
        reverse_row = not reverse_row

    global_route = [_to_global_position(point, reference, altitude_m=altitude_m) for point in _dedupe_local_points(route_local)]
    return _build_mission_items(
        global_route,
        altitude_m=altitude_m,
        hold_time_s=hold_time_s,
        acceptance_radius_m=acceptance_radius_m,
        pass_radius_m=pass_radius_m,
        yaw_deg=yaw_deg,
    )


def generate_grid_survey_route(
    geofence: Sequence[GPSPosition | tuple[float, float] | tuple[float, float, float]],
    *,
    altitude_m: float,
    grid_spacing_m: float,
    hold_time_s: float = 0.0,
    acceptance_radius_m: float = 2.0,
    pass_radius_m: float = 0.0,
    yaw_deg: float = 0.0,
) -> list[dict[str, int | float]]:
    """Generate a grid of survey waypoints across the geofence interior."""

    _validate_altitude(altitude_m)
    if grid_spacing_m <= 0:
        raise ValueError("grid_spacing_m must be greater than 0.")

    boundary = _normalize_boundary(geofence)
    reference = _projection_reference(boundary)
    local_boundary = [_to_local_xy(point, reference) for point in boundary]
    grid_angle = _principal_axis_angle(local_boundary)
    rotated_boundary = [_rotate_xy(point, -grid_angle) for point in local_boundary]

    x_values = [point[0] for point in rotated_boundary]
    y_values = [point[1] for point in rotated_boundary]
    x_positions = _grid_positions(min(x_values), max(x_values), grid_spacing_m)
    y_positions = _grid_positions(min(y_values), max(y_values), grid_spacing_m)

    route_local: list[tuple[float, float]] = []
    reverse_row = False
    for y in y_positions:
        row_points = []
        ordered_x = list(reversed(x_positions)) if reverse_row else x_positions
        for x in ordered_x:
            point = (x, y)
            if _point_in_polygon(point, rotated_boundary):
                row_points.append(_rotate_xy(point, grid_angle))
        if row_points:
            route_local.extend(row_points)
            reverse_row = not reverse_row

    global_route = [_to_global_position(point, reference, altitude_m=altitude_m) for point in _dedupe_local_points(route_local)]
    return _build_mission_items(
        global_route,
        altitude_m=altitude_m,
        hold_time_s=hold_time_s,
        acceptance_radius_m=acceptance_radius_m,
        pass_radius_m=pass_radius_m,
        yaw_deg=yaw_deg,
    )


def _build_mission_items(
    route_points: Sequence[GPSPosition],
    *,
    altitude_m: float,
    hold_time_s: float,
    acceptance_radius_m: float,
    pass_radius_m: float,
    yaw_deg: float,
) -> list[dict[str, int | float]]:
    if not route_points:
        raise ValueError("Route generation produced no waypoints inside the geofence.")

    mission_items: list[dict[str, int | float]] = []
    for index, point in enumerate(route_points):
        mission_items.append(
            MAVLinkMissionWaypoint(
                seq=index,
                current=1 if index == 0 else 0,
                param1=hold_time_s,
                param2=acceptance_radius_m,
                param3=pass_radius_m,
                param4=yaw_deg,
                x_lat=point.latitude,
                y_lon=point.longitude,
                z_alt=point.altitude_m if point.altitude_m is not None else altitude_m,
            ).to_mission_item()
        )
    return mission_items


def append_loop_jump_to_mission(
    mission_items: Sequence[dict[str, int | float]],
    *,
    jump_to_seq: int = 1,
    repeat_count: int = -1,
) -> list[dict[str, int | float]]:
    if not mission_items:
        raise ValueError("mission_items must contain at least one waypoint before a loop jump can be added.")

    if jump_to_seq < 0:
        raise ValueError("jump_to_seq must be greater than or equal to 0.")

    max_seq = max(int(item["seq"]) for item in mission_items)
    if jump_to_seq > max_seq:
        raise ValueError("jump_to_seq must refer to an existing mission sequence number.")

    looped_items = [dict(item) for item in mission_items]
    looped_items.append(
        MAVLinkMissionWaypoint(
            seq=len(looped_items),
            command=MAV_CMD_DO_JUMP,
            current=0,
            autocontinue=1,
            param1=float(jump_to_seq),
            param2=float(repeat_count),
            param3=0.0,
            param4=0.0,
            x_lat=0.0,
            y_lon=0.0,
            z_alt=0.0,
        ).to_mission_item()
    )
    return looped_items


def _validate_altitude(altitude_m: float) -> None:
    if altitude_m <= 0:
        raise ValueError("altitude_m must be greater than 0.")


def _normalize_boundary(
    geofence: Sequence[GPSPosition | tuple[float, float] | tuple[float, float, float]],
) -> list[GPSPosition]:
    if len(geofence) < 3:
        raise ValueError("A geofence requires at least three coordinates.")

    boundary = [_coerce_position(point) for point in geofence]
    deduplicated: list[GPSPosition] = []
    for point in boundary:
        if not deduplicated or not _same_position(point, deduplicated[-1]):
            deduplicated.append(point)

    if len(deduplicated) >= 2 and _same_position(deduplicated[0], deduplicated[-1]):
        deduplicated.pop()

    if len(deduplicated) < 3:
        raise ValueError("A geofence requires at least three distinct boundary coordinates.")

    return deduplicated


def _coerce_position(point: GPSPosition | tuple[float, float] | tuple[float, float, float]) -> GPSPosition:
    if isinstance(point, GPSPosition):
        return point

    if len(point) == 2:
        latitude, longitude = point
        altitude_m = None
    elif len(point) == 3:
        latitude, longitude, altitude_m = point
    else:
        raise ValueError("Each coordinate must contain latitude/longitude or latitude/longitude/altitude.")

    return GPSPosition(latitude=latitude, longitude=longitude, altitude_m=altitude_m)


def _ensure_orientation(boundary: list[GPSPosition], *, clockwise: bool) -> list[GPSPosition]:
    signed_area = _signed_area(boundary)
    is_clockwise = signed_area < 0
    if is_clockwise == clockwise:
        return boundary

    anchor = boundary[0]
    reversed_boundary = [anchor, *reversed(boundary[1:])]
    return reversed_boundary


def _signed_area(boundary: Iterable[GPSPosition]) -> float:
    points = list(boundary)
    area = 0.0
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        area += (point.longitude * next_point.latitude) - (next_point.longitude * point.latitude)
    return area / 2.0


def _same_position(left: GPSPosition, right: GPSPosition) -> bool:
    return (
        isclose(left.latitude, right.latitude, abs_tol=1e-9)
        and isclose(left.longitude, right.longitude, abs_tol=1e-9)
        and (
            left.altitude_m == right.altitude_m
            or (
                left.altitude_m is not None
                and right.altitude_m is not None
                and isclose(left.altitude_m, right.altitude_m, abs_tol=1e-6)
            )
        )
    )


def _projection_reference(boundary: Sequence[GPSPosition]) -> tuple[float, float]:
    lat = sum(point.latitude for point in boundary) / len(boundary)
    lon = sum(point.longitude for point in boundary) / len(boundary)
    return lat, lon


def _to_local_xy(point: GPSPosition, reference: tuple[float, float]) -> tuple[float, float]:
    ref_lat, ref_lon = reference
    x = radians(point.longitude - ref_lon) * EARTH_RADIUS_M * cos(radians(ref_lat))
    y = radians(point.latitude - ref_lat) * EARTH_RADIUS_M
    return x, y


def _to_global_position(point: tuple[float, float], reference: tuple[float, float], *, altitude_m: float) -> GPSPosition:
    ref_lat, ref_lon = reference
    x, y = point
    latitude = ref_lat + (y / EARTH_RADIUS_M) * (180.0 / pi)
    longitude = ref_lon + (x / (EARTH_RADIUS_M * cos(radians(ref_lat)))) * (180.0 / pi)
    return GPSPosition(latitude=latitude, longitude=longitude, altitude_m=altitude_m)


def _rotate_xy(point: tuple[float, float], angle_rad: float) -> tuple[float, float]:
    x, y = point
    cos_angle = cos(angle_rad)
    sin_angle = sin(angle_rad)
    return (
        x * cos_angle - y * sin_angle,
        x * sin_angle + y * cos_angle,
    )


def _principal_axis_angle(points: Sequence[tuple[float, float]]) -> float:
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)

    cov_xx = sum((point[0] - mean_x) ** 2 for point in points) / len(points)
    cov_yy = sum((point[1] - mean_y) ** 2 for point in points) / len(points)
    cov_xy = sum((point[0] - mean_x) * (point[1] - mean_y) for point in points) / len(points)

    return 0.5 * atan2(2.0 * cov_xy, cov_xx - cov_yy)


def _coverage_scan_lines(points: Sequence[tuple[float, float]], desired_spacing_m: float) -> list[float]:
    y_values = [point[1] for point in points]
    min_y = min(y_values)
    max_y = max(y_values)
    span = max_y - min_y
    if span <= EPSILON:
        return [(min_y + max_y) / 2.0]

    passes = max(1, ceil(span / desired_spacing_m))
    return [
        min_y + span * ((index + 0.5) / passes)
        for index in range(passes)
    ]


def _grid_positions(min_value: float, max_value: float, spacing_m: float) -> list[float]:
    span = max_value - min_value
    if span <= EPSILON:
        return [min_value]

    positions = [min_value]
    current = min_value + spacing_m
    while current < max_value - EPSILON:
        positions.append(current)
        current += spacing_m
    if max_value - positions[-1] > spacing_m * 0.25:
        positions.append(max_value)
    return positions


def _scanline_intervals(polygon: Sequence[tuple[float, float]], y: float) -> list[tuple[float, float]]:
    intersections: list[float] = []
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        x1, y1 = start
        x2, y2 = end
        if abs(y1 - y2) <= EPSILON:
            continue
        if (y1 <= y < y2) or (y2 <= y < y1):
            ratio = (y - y1) / (y2 - y1)
            intersections.append(x1 + ratio * (x2 - x1))

    intersections.sort()
    intervals: list[tuple[float, float]] = []
    for index in range(0, len(intersections) - 1, 2):
        intervals.append((intersections[index], intersections[index + 1]))
    return intervals


def _point_in_polygon(point: tuple[float, float], polygon: Sequence[tuple[float, float]]) -> bool:
    if any(_point_on_segment(point, polygon[index], polygon[(index + 1) % len(polygon)]) for index in range(len(polygon))):
        return True

    x, y = point
    inside = False
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        x1, y1 = start
        x2, y2 = end
        intersects = ((y1 > y) != (y2 > y)) and (x < ((x2 - x1) * (y - y1) / (y2 - y1 + EPSILON) + x1))
        if intersects:
            inside = not inside
    return inside


def _point_on_segment(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> bool:
    px, py = point
    x1, y1 = start
    x2, y2 = end
    cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1)
    if abs(cross) > 1e-6:
        return False

    min_x = min(x1, x2) - 1e-6
    max_x = max(x1, x2) + 1e-6
    min_y = min(y1, y2) - 1e-6
    max_y = max(y1, y2) + 1e-6
    return min_x <= px <= max_x and min_y <= py <= max_y


def _dedupe_local_points(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    deduped: list[tuple[float, float]] = []
    for point in points:
        if not deduped:
            deduped.append(point)
            continue
        prev = deduped[-1]
        if abs(prev[0] - point[0]) <= 1e-6 and abs(prev[1] - point[1]) <= 1e-6:
            continue
        deduped.append(point)
    return deduped
