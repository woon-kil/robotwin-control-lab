"""Collision geometry. 간격 공차로 겹침을 숨기지 않는다.

면이 맞닿기만 한 경우(separation == 0)는 충돌이 아니다.
길이 0인 분리축은 축으로 쓰지 않는다. 이것은 clearance가 아니다.
"""

from __future__ import annotations

import math
import struct
from pathlib import Path

from core.kinematics.transform import Matrix

Point = tuple[float, float, float]
Triangle = tuple[Point, Point, Point]

DEGENERATE_AXIS_LENGTH_SQUARED = 1e-24


def load_binary_stl(path: Path) -> tuple[Triangle, ...]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"truncated stl {path.name}")
    count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + count * 50
    if len(data) < expected:
        raise ValueError(f"truncated stl {path.name}")
    triangles: list[Triangle] = []
    offset = 84
    for _ in range(count):
        numbers = struct.unpack_from("<12fH", data, offset)
        offset += 50
        triangle = (
            (float(numbers[3]), float(numbers[4]), float(numbers[5])),
            (float(numbers[6]), float(numbers[7]), float(numbers[8])),
            (float(numbers[9]), float(numbers[10]), float(numbers[11])),
        )
        if _triangle_area_squared(triangle) > 0.0:
            triangles.append(triangle)
    if not triangles:
        raise ValueError(f"stl has no triangles {path.name}")
    return tuple(triangles)


def aabb_of_triangles(triangles: tuple[Triangle, ...]) -> tuple[Point, Point]:
    lows = [math.inf, math.inf, math.inf]
    highs = [-math.inf, -math.inf, -math.inf]
    for triangle in triangles:
        for point in triangle:
            for axis in range(3):
                lows[axis] = min(lows[axis], point[axis])
                highs[axis] = max(highs[axis], point[axis])
    return (lows[0], lows[1], lows[2]), (highs[0], highs[1], highs[2])


def transform_point(matrix: Matrix, point: Point) -> Point:
    return (
        matrix[0][0] * point[0] + matrix[0][1] * point[1] + matrix[0][2] * point[2] + matrix[0][3],
        matrix[1][0] * point[0] + matrix[1][1] * point[1] + matrix[1][2] * point[2] + matrix[1][3],
        matrix[2][0] * point[0] + matrix[2][1] * point[1] + matrix[2][2] * point[2] + matrix[2][3],
    )


def transform_triangles(matrix: Matrix, triangles: tuple[Triangle, ...]) -> tuple[Triangle, ...]:
    return tuple(
        tuple(transform_point(matrix, point) for point in triangle)
        for triangle in triangles
    )


def transformed_aabb(matrix: Matrix, low: Point, high: Point) -> tuple[Point, Point]:
    lows = [math.inf, math.inf, math.inf]
    highs = [-math.inf, -math.inf, -math.inf]
    for x in (low[0], high[0]):
        for y in (low[1], high[1]):
            for z in (low[2], high[2]):
                point = transform_point(matrix, (x, y, z))
                for axis in range(3):
                    lows[axis] = min(lows[axis], point[axis])
                    highs[axis] = max(highs[axis], point[axis])
    return (lows[0], lows[1], lows[2]), (highs[0], highs[1], highs[2])


def aabb_overlap(a_low: Point, a_high: Point, b_low: Point, b_high: Point) -> bool:
    return all(a_high[axis] > b_low[axis] and b_high[axis] > a_low[axis] for axis in range(3))


def point_in_aabb(point: Point, low: Point, high: Point) -> bool:
    return all(low[axis] <= point[axis] <= high[axis] for axis in range(3))


def triangle_intersects_aabb(triangle: Triangle, low: Point, high: Point) -> bool:
    tri_low, tri_high = _triangle_aabb(triangle)
    if not aabb_overlap(tri_low, tri_high, low, high):
        return False
    center = tuple((low[axis] + high[axis]) * 0.5 for axis in range(3))
    half = tuple((high[axis] - low[axis]) * 0.5 for axis in range(3))
    points = tuple(_sub(point, center) for point in triangle)
    box_axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    for axis in box_axes:
        if _separated_from_centered_box(axis, points, half):
            return False
    edges = _edges(points)
    normal = _cross(edges[0], edges[1])
    if _dot(normal, normal) > DEGENERATE_AXIS_LENGTH_SQUARED and _separated_from_centered_box(normal, points, half):
        return False
    for edge in edges:
        for box_axis in box_axes:
            axis = _cross(edge, box_axis)
            if _dot(axis, axis) <= DEGENERATE_AXIS_LENGTH_SQUARED:
                continue
            if _separated_from_centered_box(axis, points, half):
                return False
    return True


def triangles_intersect(left: Triangle, right: Triangle) -> bool:
    left_edges = _edges(left)
    right_edges = _edges(right)
    axes = [_cross(left_edges[0], left_edges[1]), _cross(right_edges[0], right_edges[1])]
    for left_edge in left_edges:
        for right_edge in right_edges:
            axes.append(_cross(left_edge, right_edge))
    for axis in axes:
        if _dot(axis, axis) <= DEGENERATE_AXIS_LENGTH_SQUARED:
            continue
        left_min, left_max = _project_interval(axis, left)
        right_min, right_max = _project_interval(axis, right)
        # 같은 평면의 법선 투영은 두께가 0이다. 그 축은 분리축이 아니다.
        if _coplanar_projection(axis, left_min, left_max, right_min, right_max):
            continue
        if not _strict_overlap(left_min, left_max, right_min, right_max):
            return False
    return True


def mesh_intersects_aabb(triangles: tuple[Triangle, ...], low: Point, high: Point) -> bool:
    for triangle in triangles:
        if triangle_intersects_aabb(triangle, low, high):
            return True
    return False


def mesh_intersects_mesh(
    left: tuple[Triangle, ...],
    right: tuple[Triangle, ...],
    cell_m: float,
) -> tuple[bool, int]:
    """겹치는 grid cell의 삼각형만 검사한다. 반환은 (충돌, 삼각형 쌍 검사 수)."""
    if cell_m <= 0.0:
        raise ValueError("broadphase cell must be positive")
    grid: dict[tuple[int, int, int], list[int]] = {}
    for index, triangle in enumerate(left):
        for key in _cells(triangle, cell_m):
            grid.setdefault(key, []).append(index)
    tests = 0
    for triangle in right:
        seen: set[int] = set()
        for key in _cells(triangle, cell_m):
            for index in grid.get(key, ()):
                if index in seen:
                    continue
                seen.add(index)
                tests += 1
                if triangles_intersect(left[index], triangle):
                    return True, tests
    return False, tests


def obb_intersects_aabb(
    center: Point,
    axes: tuple[Point, Point, Point],
    half: Point,
    low: Point,
    high: Point,
) -> bool:
    box_center = tuple((low[axis] + high[axis]) * 0.5 for axis in range(3))
    box_half = tuple((high[axis] - low[axis]) * 0.5 for axis in range(3))
    box_axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    test_axes = list(axes) + list(box_axes)
    for left_axis in axes:
        for right_axis in box_axes:
            test_axes.append(_cross(left_axis, right_axis))
    for axis in test_axes:
        if _dot(axis, axis) <= DEGENERATE_AXIS_LENGTH_SQUARED:
            continue
        if _separated_bodies(axis, center, half, axes, box_center, box_half, box_axes):
            return False
    return True


def obb_corners(center: Point, axes: tuple[Point, Point, Point], half: Point) -> tuple[Point, ...]:
    corners: list[Point] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                signs = (sx, sy, sz)
                point = center
                for index in range(3):
                    point = _add(point, _scale(axes[index], signs[index] * half[index]))
                corners.append(point)
    return tuple(corners)


def _separated_bodies(
    axis: Point,
    center: Point,
    half: Point,
    axes: tuple[Point, Point, Point],
    box_center: Point,
    box_half: Point,
    box_axes: tuple[Point, Point, Point],
) -> bool:
    distance = abs(_dot(axis, _sub(box_center, center)))
    radius = _body_radius(axis, axes, half) + _body_radius(axis, box_axes, box_half)
    return distance >= radius


def _body_radius(axis: Point, axes: tuple[Point, Point, Point], half: Point) -> float:
    return sum(half[index] * abs(_dot(axes[index], axis)) for index in range(3))


def _separated_from_centered_box(axis: Point, points: tuple[Point, ...], half: Point) -> bool:
    low, high = _project_interval(axis, points)
    radius = half[0] * abs(axis[0]) + half[1] * abs(axis[1]) + half[2] * abs(axis[2])
    return not _strict_overlap(low, high, -radius, radius)


def _strict_overlap(left_min: float, left_max: float, right_min: float, right_max: float) -> bool:
    return left_max > right_min and right_max > left_min


def _coplanar_projection(
    axis: Point,
    left_min: float,
    left_max: float,
    right_min: float,
    right_max: float,
) -> bool:
    tolerance = math.sqrt(_dot(axis, axis)) * 1e-9
    return (
        left_max - left_min <= tolerance
        and right_max - right_min <= tolerance
        and abs(left_min - right_min) <= tolerance
    )


def _project_interval(axis: Point, points: tuple[Point, ...]) -> tuple[float, float]:
    dots = [_dot(axis, point) for point in points]
    return min(dots), max(dots)


def _triangle_aabb(triangle: Triangle) -> tuple[Point, Point]:
    lows = [min(triangle[0][axis], triangle[1][axis], triangle[2][axis]) for axis in range(3)]
    highs = [max(triangle[0][axis], triangle[1][axis], triangle[2][axis]) for axis in range(3)]
    return (lows[0], lows[1], lows[2]), (highs[0], highs[1], highs[2])


def _cells(triangle: Triangle, cell_m: float) -> tuple[tuple[int, int, int], ...]:
    low, high = _triangle_aabb(triangle)
    starts = [math.floor(low[axis] / cell_m) for axis in range(3)]
    stops = [math.floor(high[axis] / cell_m) for axis in range(3)]
    keys: list[tuple[int, int, int]] = []
    for ix in range(starts[0], stops[0] + 1):
        for iy in range(starts[1], stops[1] + 1):
            for iz in range(starts[2], stops[2] + 1):
                keys.append((ix, iy, iz))
    return tuple(keys)


def _edges(points: tuple[Point, ...]) -> tuple[Point, Point, Point]:
    return (
        _sub(points[1], points[0]),
        _sub(points[2], points[1]),
        _sub(points[0], points[2]),
    )


def _triangle_area_squared(triangle: Triangle) -> float:
    normal = _cross(_sub(triangle[1], triangle[0]), _sub(triangle[2], triangle[0]))
    return _dot(normal, normal)


def _add(left: Point, right: Point) -> Point:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _sub(left: Point, right: Point) -> Point:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def _scale(point: Point, factor: float) -> Point:
    return (point[0] * factor, point[1] * factor, point[2] * factor)


def _dot(left: Point, right: Point) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _cross(left: Point, right: Point) -> Point:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )
