"""UR10e collision STL을 P-001B FK frame에 붙인다. 새 FK solver는 만들지 않는다."""

from __future__ import annotations

import math
from dataclasses import dataclass

from core.kinematics.fk import chain_frames
from core.kinematics.transform import Matrix, multiply, origin_matrix
from core.model.types import RobotModel
from core.model.ur10e import _load_nested, source_root
from validator.geometry import (
    Point,
    Triangle,
    aabb_of_triangles,
    aabb_overlap,
    load_binary_stl,
    mesh_intersects_aabb,
    mesh_intersects_mesh,
    obb_corners,
    obb_intersects_aabb,
    point_in_aabb,
    transform_point,
    transform_triangles,
    transformed_aabb,
)
from workcell.layout import tool_collision_matrix_in_wrist3

LINK_ORDER = (
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
)

# ur_macro.xacro legacy branch. visual yaml에 mesh_offset가 없으면 이 origin을 쓴다.
_LEGACY = (
    ("base", "base_link_inertia", "xyz", (0.0, 0.0, 0.0), (0.0, 0.0, math.pi)),
    ("shoulder", "shoulder_link", "xyz", (0.0, 0.0, 0.0), (0.0, 0.0, math.pi)),
    ("upper_arm", "upper_arm_link", "shoulder_offset", None, (math.pi / 2.0, 0.0, -math.pi / 2.0)),
    ("forearm", "forearm_link", "elbow_offset", None, (math.pi / 2.0, 0.0, -math.pi / 2.0)),
    ("wrist_1", "wrist_1_link", "visual_offset", None, (math.pi / 2.0, 0.0, 0.0)),
    ("wrist_2", "wrist_2_link", "visual_offset", None, (0.0, 0.0, 0.0)),
    ("wrist_3", "wrist_3_link", "visual_offset_xyz", None, (math.pi / 2.0, 0.0, 0.0)),
)


@dataclass(frozen=True)
class LinkMesh:
    name: str
    link: str
    triangles: tuple[Triangle, ...]
    local_low: Point
    local_high: Point
    origin_xyz: Point
    origin_rpy: Point


@dataclass(frozen=True)
class Hit:
    a: str
    b: str


class CollisionScene:
    def __init__(self, workcell: dict) -> None:
        self.links = load_collision_links()
        self.tool_in_wrist = tool_collision_matrix_in_wrist3(workcell)
        size = workcell["tool_placeholder"]["size_m"]
        self.tool_half = (float(size[0]) * 0.5, float(size[1]) * 0.5, float(size[2]) * 0.5)
        self._by_link = {link.link: link for link in self.links}

    def evaluate(self, model: RobotModel, joints: tuple[float, ...], resolved: dict, config: dict) -> dict:
        posed = self._pose(model, joints, resolved, config)
        cell = float(config["collision"]["broadphase_cell_m"])
        robot_hits: list[Hit] = []
        separated = 0
        narrow_pairs = 0
        triangle_tests = 0
        world_meshes: dict[str, tuple[Triangle, ...]] = {}
        for link in self.links:
            low, high = posed["link_aabbs"][link.link]
            overlapped = [box for box in posed["boxes"] if aabb_overlap(low, high, box[1], box[2])]
            separated += len(posed["boxes"]) - len(overlapped)
            if not overlapped:
                continue
            world_meshes[link.link] = transform_triangles(posed["link_matrices"][link.link], link.triangles)
            for name, box_low, box_high in overlapped:
                narrow_pairs += 1
                triangle_tests += len(link.triangles)
                if mesh_intersects_aabb(world_meshes[link.link], box_low, box_high):
                    robot_hits.append(Hit(link.link, name))
        tool_hits = [Hit("tool_placeholder", name) for name, _low, _high in posed["tool_hits"]]
        parent = str(config["collision"]["tool_parent_link_excluded"])
        tool_robot: list[Hit] = []
        for link in self.links:
            if link.link == parent:
                continue
            low, high = posed["link_aabbs"][link.link]
            if not aabb_overlap(low, high, posed["tool_low"], posed["tool_high"]):
                separated += 1
                continue
            narrow_pairs += 1
            if link.link not in world_meshes:
                world_meshes[link.link] = transform_triangles(posed["link_matrices"][link.link], link.triangles)
            local = _world_triangles_to_tool(posed["tool_inverse"], world_meshes[link.link])
            triangle_tests += len(local)
            if mesh_intersects_aabb(local, _negative(self.tool_half), self.tool_half):
                tool_robot.append(Hit("tool_placeholder", link.link))
        self_hits: list[Hit] = []
        order = [link.link for link in self.links]
        for left_index, left_name in enumerate(order):
            for right_name in order[left_index + 2 :]:
                if not aabb_overlap(
                    posed["link_aabbs"][left_name][0],
                    posed["link_aabbs"][left_name][1],
                    posed["link_aabbs"][right_name][0],
                    posed["link_aabbs"][right_name][1],
                ):
                    separated += 1
                    continue
                narrow_pairs += 1
                if left_name not in world_meshes:
                    world_meshes[left_name] = transform_triangles(
                        posed["link_matrices"][left_name], self._by_link[left_name].triangles
                    )
                if right_name not in world_meshes:
                    world_meshes[right_name] = transform_triangles(
                        posed["link_matrices"][right_name], self._by_link[right_name].triangles
                    )
                hit, tests = mesh_intersects_mesh(world_meshes[left_name], world_meshes[right_name], cell)
                triangle_tests += tests
                if hit:
                    self_hits.append(Hit(left_name, right_name))
        return {
            "tcp_m": list(posed["tcp"]),
            "in_prohibited_zone": posed["in_zone"],
            "prohibited_zone_m": posed["zone"],
            "robot_hits": [_hit(item) for item in robot_hits],
            "tool_hits": [_hit(item) for item in tool_hits],
            "tool_robot_hits": [_hit(item) for item in tool_robot],
            "self_hits": [_hit(item) for item in self_hits],
            "aabb_separated_pairs": separated,
            "narrow_phase_pairs": narrow_pairs,
            "triangle_tests": triangle_tests,
        }

    def screen(self, model: RobotModel, joints: tuple[float, ...], resolved: dict, config: dict) -> dict:
        posed = self._pose(model, joints, resolved, config)
        return {
            "tcp_m": list(posed["tcp"]),
            "in_prohibited_zone": posed["in_zone"],
            "tool_hits": [_hit(Hit("tool_placeholder", name)) for name, _low, _high in posed["tool_hits"]],
            "joint_position": list(joints),
        }

    def _pose(self, model: RobotModel, joints: tuple[float, ...], resolved: dict, config: dict) -> dict:
        base = origin_matrix(
            tuple(resolved["T_world_robot_base"]["xyz_m"]),
            tuple(resolved["T_world_robot_base"]["rpy_rad"]),
        )
        frames = {str(frame["child"]): frame["cumulative_matrix"] for frame in chain_frames(model, joints)}
        if any(link.link not in frames for link in self.links) or "tool0" not in frames:
            raise ValueError("FK chain is missing a collision link")
        link_matrices = {
            link.link: multiply(multiply(base, frames[link.link]), origin_matrix(link.origin_xyz, link.origin_rpy))
            for link in self.links
        }
        link_aabbs = {
            link.link: transformed_aabb(link_matrices[link.link], link.local_low, link.local_high)
            for link in self.links
        }
        tool_world = multiply(multiply(base, frames["wrist_3_link"]), self.tool_in_wrist)
        tool_center = (tool_world[0][3], tool_world[1][3], tool_world[2][3])
        tool_axes = (
            _column(tool_world, 0),
            _column(tool_world, 1),
            _column(tool_world, 2),
        )
        tcp = transform_point(base, _translation(frames["tool0"]))
        boxes = environment_boxes(resolved)
        tool_hits = [
            (name, low, high)
            for name, low, high in boxes
            if obb_intersects_aabb(tool_center, tool_axes, self.tool_half, low, high)
        ]
        corners = obb_corners(tool_center, tool_axes, self.tool_half)
        tool_low = tuple(min(point[axis] for point in corners) for axis in range(3))
        tool_high = tuple(max(point[axis] for point in corners) for axis in range(3))
        zone = prohibited_zone(resolved, config)
        return {
            "tcp": tcp,
            "link_matrices": link_matrices,
            "link_aabbs": link_aabbs,
            "boxes": boxes,
            "tool_hits": tool_hits,
            "tool_low": tool_low,
            "tool_high": tool_high,
            "tool_inverse": _invert_rigid(tool_world),
            "zone": zone,
            "in_zone": point_in_aabb(tcp, zone[0], zone[1]),
        }


def load_collision_links() -> tuple[LinkMesh, ...]:
    root = source_root()
    physical = _load_nested(root / "config" / "ur10e" / "physical_parameters.yaml")
    visual = _load_nested(root / "config" / "ur10e" / "visual_parameters.yaml")
    meshes = visual["mesh_files"]
    offsets = physical["offsets"]
    loaded: list[LinkMesh] = []
    for mesh_name, link, mode, fixed_xyz, rpy in _LEGACY:
        entry = meshes[mesh_name]
        if "mesh_offset" in entry:
            raise ValueError(mesh_name + " has mesh_offset and the legacy origin is not the source")
        xyz = _origin_xyz(entry, mode, fixed_xyz, offsets)
        path = root / _unquote(str(entry["collision"]["mesh"]["path"]))
        triangles = load_binary_stl(path)
        low, high = aabb_of_triangles(triangles)
        loaded.append(
            LinkMesh(
                name=mesh_name,
                link=link,
                triangles=triangles,
                local_low=low,
                local_high=high,
                origin_xyz=xyz,
                origin_rpy=rpy,
            )
        )
    return tuple(loaded)


def environment_boxes(resolved: dict) -> tuple[tuple[str, Point, Point], ...]:
    boxes: list[tuple[str, Point, Point]] = [
        ("floor", _box(resolved["floor_world"]["aabb_m"])[0], _box(resolved["floor_world"]["aabb_m"])[1]),
        ("pedestal", _box(resolved["fixture_world"]["aabb_m"])[0], _box(resolved["fixture_world"]["aabb_m"])[1]),
    ]
    for post in resolved["posts_world"]:
        low, high = _box(post["aabb_m"])
        boxes.append((str(post["id"]), low, high))
    panel_low, panel_high = _box(resolved["bolt_panel_world"]["aabb_m"])
    boxes.append(("bolt_panel", panel_low, panel_high))
    return tuple(boxes)


def prohibited_zone(resolved: dict, config: dict) -> tuple[Point, Point]:
    zone = config["safety_zone"]
    mode = str(zone["mode"])
    if mode == "explicit_aabb":
        explicit = zone["explicit_aabb_m"]
        if not isinstance(explicit, dict):
            raise ValueError("explicit safety aabb is missing")
        return _box(explicit)
    if mode != "beyond_bolt_panel_positive_x":
        raise ValueError("unsupported safety zone mode")
    low, high = _box(resolved["bolt_panel_world"]["aabb_m"])
    depth = float(zone["prohibited_depth_m"])
    if not math.isfinite(depth) or depth <= 0.0:
        raise ValueError("prohibited depth must be finite and positive")
    return (high[0], low[1], low[2]), (high[0] + depth, high[1], high[2])


def _origin_xyz(entry: dict, mode: str, fixed: Point | None, offsets: dict) -> Point:
    if mode == "xyz":
        if fixed is None:
            raise ValueError("fixed mesh origin is missing")
        return fixed
    if mode == "shoulder_offset":
        return (0.0, 0.0, float(offsets["shoulder_offset"]))
    if mode == "elbow_offset":
        return (0.0, 0.0, float(offsets["elbow_offset"]))
    if mode == "visual_offset":
        return (0.0, 0.0, float(entry["visual_offset"]))
    if mode == "visual_offset_xyz":
        text = _unquote(str(entry["visual_offset_xyz"]))
        parts = text.split()
        if len(parts) != 3:
            raise ValueError("wrist_3 visual_offset_xyz is not three numbers")
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    raise ValueError("unknown mesh origin mode")


def _box(aabb: dict) -> tuple[Point, Point]:
    low = aabb["min_m"]
    high = aabb["max_m"]
    return (float(low[0]), float(low[1]), float(low[2])), (float(high[0]), float(high[1]), float(high[2]))


def _hit(item: Hit) -> dict[str, str]:
    return {"a": item.a, "b": item.b}


def _column(matrix: Matrix, index: int) -> Point:
    return (matrix[0][index], matrix[1][index], matrix[2][index])


def _translation(matrix: Matrix) -> Point:
    return (matrix[0][3], matrix[1][3], matrix[2][3])


def _negative(point: Point) -> Point:
    return (-point[0], -point[1], -point[2])


def _unquote(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        return value[1:-1]
    return value


def _world_triangles_to_tool(inverse: Matrix, triangles: tuple[Triangle, ...]) -> tuple[Triangle, ...]:
    return transform_triangles(inverse, triangles)


def _invert_rigid(matrix: Matrix) -> Matrix:
    rotation = [[matrix[row][col] for col in range(3)] for row in range(3)]
    translation = [matrix[0][3], matrix[1][3], matrix[2][3]]
    transposed = [[rotation[col][row] for col in range(3)] for row in range(3)]
    inverse_translation = [
        -sum(transposed[row][col] * translation[col] for col in range(3)) for row in range(3)
    ]
    inverse = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for row in range(3):
        for col in range(3):
            inverse[row][col] = transposed[row][col]
        inverse[row][3] = inverse_translation[row]
    return inverse
