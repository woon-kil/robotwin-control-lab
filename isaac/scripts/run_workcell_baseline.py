"""Headless Isaac baseline for the UR10e reference workcell.

Writes evidence/P-005 only. Does not construct RigidPrim and does not retune P-004 drives.
"""

from __future__ import annotations

import argparse
import math
import platform
import subprocess
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--repo", required=True)
parser.add_argument("--isaac-root", default=str(Path.home() / "isaacsim"))
parser.add_argument("--evidence-dir", default="")
args = parser.parse_args()

REPO = Path(args.repo).resolve()
ISAAC_ROOT = Path(args.isaac_root).resolve()
sys.path.insert(0, str(REPO))

from core.kinematics.transform import matrix_to_quaternion  # noqa: E402
from isaac.ur10e_commands import map_canonical_dofs, validate_joint_targets  # noqa: E402
from workcell.layout import (  # noqa: E402
    load_workcell,
    max_position_difference_m,
    resolve_workcell,
    tool_collision_matrix_in_wrist3,
)
from workcell.report import judge_workcell, render_summary, render_test_results, write_evidence  # noqa: E402

EVIDENCE = Path(args.evidence_dir).resolve() if args.evidence_dir else REPO / "evidence" / "P-005"
if EVIDENCE.parts[-2:] != ("evidence", "P-005"):
    raise SystemExit("refusing to write outside evidence/P-005")

sys.argv = [sys.argv[0]]

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import usdrt  # noqa: E402
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.asset.importer.utils.impl.asset_utils import fix_articulation_root_for_fixed_base  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.experimental.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from omni.physx import get_physx_scene_query_interface, get_physx_simulation_interface  # noqa: E402
from omni.physx.bindings._physx import ContactEventType  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdUtils  # noqa: E402

CONFIG = load_workcell()
RESOLVED = resolve_workcell(CONFIG)
IMPORTER = CONFIG["isaac_import"]
PHYSICS = CONFIG["physics"]
USD_DIR = REPO / "isaac" / "generated" / "workcell"
URDF_PATH = REPO / "robot_description" / "generated" / CONFIG["robot"]["source_commit"] / "ur10e.urdf"
PACKAGE_ROOT = REPO / "robot_description" / "upstream" / CONFIG["robot"]["source_commit"]
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
TIMEOUT_SECONDS = 8.0
SETTLED_STEPS = 20
VELOCITY_EPSILON = 0.001
POSITION_DELTA_EPSILON = 1e-5
POSE_API = "UsdGeom.XformCache.GetLocalToWorldTransform Gf.Quatd real then imaginary"


def numbers(value) -> list[float]:
    array = value.numpy() if hasattr(value, "numpy") else np.asarray(value)
    return [float(item) for item in np.reshape(array, -1)]


def unique_geometry_prim(stage, name: str) -> str:
    matches = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.GetName() == name and str(prim.GetPath()).startswith("/ur10e/Geometry/")
    ]
    if len(matches) != 1:
        raise RuntimeError("expected one " + name + ", found " + str(matches))
    return matches[0]


def xform_pose(stage, path: str) -> dict[str, object]:
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise RuntimeError("missing prim " + path)
    if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() == "tool0":
        raise RuntimeError("tool0 gained RigidBodyAPI")
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    matrix = cache.GetLocalToWorldTransform(prim)
    transform = Gf.Transform()
    transform.SetMatrix(matrix)
    translation = transform.GetTranslation()
    quat = transform.GetRotation().GetQuat()
    imaginary = quat.GetImaginary()
    return {
        "frame": "isaac_world",
        "quaternion_order": "wxyz",
        "pose_api": POSE_API,
        "prim_path": path,
        "rigid_body_api": bool(prim.HasAPI(UsdPhysics.RigidBodyAPI)),
        "position_m": {"x": float(translation[0]), "y": float(translation[1]), "z": float(translation[2])},
        "quaternion_wxyz": {
            "w": float(quat.GetReal()),
            "x": float(imaginary[0]),
            "y": float(imaginary[1]),
            "z": float(imaginary[2]),
        },
    }


def position_m(pose: dict) -> list[float]:
    point = pose["position_m"]
    return [float(point["x"]), float(point["y"]), float(point["z"])]


def distance_m(left: list[float], right: list[float]) -> float:
    return math.dist(left, right)


def finite_pose(pose: dict) -> bool:
    values = list(pose["position_m"].values()) + list(pose["quaternion_wxyz"].values())
    return all(math.isfinite(float(value)) for value in values)


def ensure_gravity(stage) -> dict[str, object]:
    scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if not scenes:
        scene_prim = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        direction = PHYSICS["gravity_direction"]
        scene_prim.CreateGravityDirectionAttr().Set(Gf.Vec3f(*direction))
        scene_prim.CreateGravityMagnitudeAttr().Set(float(PHYSICS["gravity_magnitude_m_s2"]))
        scenes = [scene_prim.GetPrim()]
    scene = UsdPhysics.Scene(scenes[0])
    direction = scene.GetGravityDirectionAttr().Get()
    magnitude = scene.GetGravityMagnitudeAttr().Get()
    return {
        "prim": str(scenes[0].GetPath()),
        "direction": [float(direction[0]), float(direction[1]), float(direction[2])],
        "magnitude_m_s2": float(magnitude),
        "enabled": True,
    }


def define_xform(stage, path: str, translation: list[float]):
    prim = UsdGeom.Xform.Define(stage, path)
    prim.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translation))
    return prim


def define_box(stage, path: str, translation: list[float], size: list[float], collide: bool):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    # scale을 translate보다 앞에 두면 translation이 scale된다. 측정된 tool center 오차는 0.0846 m였다.
    translate = cube.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    scale = cube.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
    translate.Set(Gf.Vec3d(*translation))
    scale.Set(Gf.Vec3d(*size))
    cube.SetXformOpOrder([translate, scale])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def define_oriented_box(stage, path: str, matrix: list[list[float]], size: list[float]) -> None:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    qx, qy, qz, qw = matrix_to_quaternion(matrix)
    translate = cube.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    orient = cube.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    scale = cube.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
    translate.Set(Gf.Vec3d(matrix[0][3], matrix[1][3], matrix[2][3]))
    orient.Set(Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz)))
    scale.Set(Gf.Vec3d(*size))
    cube.SetXformOpOrder([translate, orient, scale])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def translate_robot(stage, translation: list[float]) -> None:
    prim = stage.GetPrimAtPath("/ur10e")
    if not prim:
        raise RuntimeError("imported robot prim /ur10e is missing")
    xform = UsdGeom.Xformable(prim)
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    current = Gf.Transform()
    current.SetMatrix(matrix)
    current_translation = current.GetTranslation()
    rotation = current.GetRotation().GetQuat()
    imaginary = rotation.GetImaginary()
    origin = all(abs(float(current_translation[axis])) <= 1.0e-6 for axis in range(3))
    identity_rotation = abs(float(rotation.GetReal())) >= 0.999999 and all(
        abs(float(imaginary[axis])) <= 1.0e-6 for axis in range(3)
    )
    if not origin or not identity_rotation:
        raise RuntimeError("imported /ur10e root transform is not identity")
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translation))


def add_workcell(stage, paths: dict[str, str]) -> None:
    UsdGeom.Xform.Define(stage, "/World")
    define_box(stage, CONFIG["floor"]["prim"], CONFIG["floor"]["center_m"], CONFIG["floor"]["size_m"], True)
    UsdGeom.Xform.Define(stage, "/World/Fixture")
    define_box(stage, CONFIG["fixture"]["prim"], CONFIG["fixture"]["center_m"], CONFIG["fixture"]["size_m"], True)
    define_xform(stage, "/World/RobotBase", CONFIG["robot"]["T_world_robot_base"]["xyz_m"])
    define_xform(stage, CONFIG["workcell"]["prim"], CONFIG["workcell"]["T_world_workcell"]["xyz_m"])
    UsdGeom.Xform.Define(stage, CONFIG["work_structure"]["prim"])
    for post in CONFIG["work_structure"]["posts"]:
        define_box(stage, post["prim"], post["center_in_workcell_m"], post["size_m"], True)
    panel = CONFIG["bolt_panel"]
    define_xform(stage, panel["prim"], panel["center_in_workcell_m"])
    define_box(stage, panel["prim"] + "/PanelSolid", [0.0, 0.0, 0.0], panel["size_m"], True)
    for target in CONFIG["bolt_targets"]:
        define_xform(stage, panel["prim"] + "/" + target["id"], target["position_m"])
    tool = CONFIG["tool_placeholder"]
    define_xform(stage, paths["tool0"] + "/" + tool["frame_name"], tool["center_in_tool0_m"])
    define_oriented_box(
        stage,
        paths["wrist_3_link"] + "/" + tool["collision_name"],
        tool_collision_matrix_in_wrist3(CONFIG),
        tool["size_m"],
    )


def collision_counts(stage, paths: dict[str, str]) -> dict[str, int]:
    def count(prefix: str) -> int:
        return sum(
            1
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(prefix) and prim.HasAPI(UsdPhysics.CollisionAPI)
        )

    tool_path = paths["wrist_3_link"] + "/" + CONFIG["tool_placeholder"]["collision_name"]
    return {
        "robot": count("/ur10e"),
        "floor": count(CONFIG["floor"]["prim"]),
        "pedestal": count(CONFIG["fixture"]["prim"]),
        "bolt_panel": count(CONFIG["bolt_panel"]["prim"]),
        "posts": count(CONFIG["work_structure"]["posts"][0]["prim"])
        + count(CONFIG["work_structure"]["posts"][1]["prim"]),
        "tool_placeholder": count(tool_path),
    }


def snapshot_positions(stage, paths: dict[str, str]) -> dict[str, list[float]]:
    positions = {
        "robot_base": position_m(xform_pose(stage, paths["base_link"])),
        "bolt_panel": position_m(xform_pose(stage, CONFIG["bolt_panel"]["prim"])),
    }
    for target in CONFIG["bolt_targets"]:
        positions[target["id"]] = position_m(xform_pose(stage, CONFIG["bolt_panel"]["prim"] + "/" + target["id"]))
    return positions


def build_once() -> dict[str, object]:
    USD_DIR.mkdir(parents=True, exist_ok=True)
    importer = URDFImporterConfig(
        urdf_path=str(URDF_PATH),
        usd_path=str(USD_DIR),
        merge_fixed_joints=bool(IMPORTER["merge_fixed_joints"]),
        merge_mesh=bool(IMPORTER["merge_mesh"]),
        collision_from_visuals=bool(IMPORTER["collision_from_visuals"]),
        allow_self_collision=bool(IMPORTER["allow_self_collision"]),
        ros_package_paths=[{"name": "ur_description", "path": str(PACKAGE_ROOT)}],
        fix_base=bool(IMPORTER["fix_base"]),
        joint_drive_type=str(IMPORTER["joint_drive_type"]),
        joint_target_type=str(IMPORTER["joint_target_type"]),
        override_joint_stiffness=float(IMPORTER["override_joint_stiffness_nm_per_rad"]),
        override_joint_damping=float(IMPORTER["override_joint_damping"]),
        run_asset_transformer=bool(IMPORTER["run_asset_transformer"]),
        run_multi_physics_conversion=bool(IMPORTER["run_multi_physics_conversion"]),
    )
    generated = URDFImporter(importer).import_urdf()
    opened, stage = stage_utils.open_stage(generated)
    if not opened or stage is None:
        raise RuntimeError("failed to open generated USD " + generated)
    gravity = ensure_gravity(stage)
    roots_before = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    relocated = int(fix_articulation_root_for_fixed_base(stage))
    roots = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError("expected one articulation root, found " + str(roots))
    paths = {
        "base_link": unique_geometry_prim(stage, "base_link"),
        "tool0": unique_geometry_prim(stage, "tool0"),
        "wrist_3_link": unique_geometry_prim(stage, "wrist_3_link"),
    }
    translate_robot(stage, CONFIG["robot"]["T_world_robot_base"]["xyz_m"])
    add_workcell(stage, paths)
    return {
        "stage": stage,
        "generated_usd": generated,
        "gravity": gravity,
        "roots_before": roots_before,
        "relocated": relocated,
        "root": roots[0],
        "paths": paths,
        "positions": snapshot_positions(stage, paths),
        "collision_counts": collision_counts(stage, paths),
    }


def regulate(robot, dof_indices: list[int], targets: tuple[float, ...]) -> None:
    checked = validate_joint_targets(targets)
    values = np.asarray(checked, dtype=np.float64).reshape(1, -1)
    zeros = np.zeros_like(values)
    robot.set_dof_velocities(zeros, dof_indices=dof_indices)
    robot.set_dof_positions(values, dof_indices=dof_indices)
    robot.set_dof_position_targets(values, dof_indices=dof_indices)
    robot.set_dof_velocity_targets(zeros, dof_indices=dof_indices)


def settle(robot, dof_indices: list[int], dt: float) -> tuple[list[float], int, bool]:
    previous = numbers(robot.get_dof_positions(dof_indices=dof_indices))
    stable = 0
    steps = 0
    elapsed = 0.0
    actual = previous
    while elapsed < TIMEOUT_SECONDS:
        simulation_app.update()
        steps += 1
        elapsed += dt
        actual = numbers(robot.get_dof_positions(dof_indices=dof_indices))
        velocity = numbers(robot.get_dof_velocities(dof_indices=dof_indices))
        delta = max(abs(actual[index] - previous[index]) for index in range(len(actual)))
        speed = max(abs(item) for item in velocity)
        if speed < VELOCITY_EPSILON and delta < POSITION_DELTA_EPSILON:
            stable += 1
        else:
            stable = 0
        previous = actual
        if stable >= SETTLED_STEPS:
            return actual, steps, False
    return actual, steps, True


def dof_setup(robot) -> tuple[list[int], list[str]]:
    mapping = map_canonical_dofs(list(robot.dof_names))
    indices = [int(numbers(robot.get_dof_indices(str(row["isaac_dof_name"])))[0]) for row in mapping]
    names = [str(row["canonical_name"]) for row in mapping]
    return indices, names


def state_snapshot(stage, robot, dof_indices: list[int], names: list[str], paths: dict[str, str]) -> dict:
    joints = numbers(robot.get_dof_positions(dof_indices=dof_indices))
    return {
        "robot_base": xform_pose(stage, paths["base_link"]),
        "tool0": xform_pose(stage, paths["tool0"]),
        "floor": xform_pose(stage, CONFIG["floor"]["prim"]),
        "pedestal": xform_pose(stage, CONFIG["fixture"]["prim"]),
        "work_structure": xform_pose(stage, CONFIG["work_structure"]["prim"]),
        "bolt_panel": xform_pose(stage, CONFIG["bolt_panel"]["prim"]),
        "bolt_targets": {
            target["id"]: xform_pose(stage, CONFIG["bolt_panel"]["prim"] + "/" + target["id"])
            for target in CONFIG["bolt_targets"]
        },
        "joint_names": names,
        "joint_rad": joints,
    }


def non_finite_count(snapshot: dict) -> int:
    count = 0
    for name in ("robot_base", "tool0", "floor", "pedestal", "work_structure", "bolt_panel"):
        if not finite_pose(snapshot[name]):
            count += 1
    for pose in snapshot["bolt_targets"].values():
        if not finite_pose(pose):
            count += 1
    if any(not math.isfinite(value) for value in snapshot["joint_rad"]):
        count += 1
    return count


def enable_contact_reports(stage) -> int:
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    usdrt_stage = usdrt.Usd.Stage.Attach(stage_id)
    count = 0
    for prim_path in usdrt_stage.GetPrimsWithAppliedAPIName("PhysicsRigidBodyAPI"):
        prim = stage.GetPrimAtPath(str(prim_path))
        if not prim:
            continue
        report = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        report.CreateThresholdAttr().Set(0.0)
        count += 1
    return count


def read_contacts() -> dict[str, object]:
    report = get_physx_simulation_interface().get_contact_report()
    headers = report[0]
    data = report[1]
    pairs = []
    unexpected = 0
    penetrations = 0
    self_pairs = 0
    for header in headers:
        if header.type == ContactEventType.CONTACT_LOST:
            continue
        collider0 = str(PhysicsSchemaTools.intToSdfPath(header.collider0))
        collider1 = str(PhysicsSchemaTools.intToSdfPath(header.collider1))
        separations = [
            float(data[header.contact_data_offset + index].separation) for index in range(int(header.num_contact_data))
        ]
        self_pair = "/ur10e" in collider0 and "/ur10e" in collider1
        min_separation = min(separations) if separations else None
        row = {
            "collider0": collider0,
            "collider1": collider1,
            "event": str(header.type),
            "separations_m": separations,
            "min_separation_m": min_separation,
            "self_pair": self_pair,
        }
        pairs.append(row)
        if self_pair:
            self_pairs += 1
            continue
        if min_separation is None or min_separation <= 0.0:
            unexpected += 1
        penetrations += sum(1 for value in separations if value < 0.0)
    return {
        "api": "omni.physx.get_physx_simulation_interface().get_contact_report",
        "separation_definition": "ContactData.separation as returned. separation < 0 counts as penetration. A contact event with separation > 0 is recorded and is not an unexpected collision.",
        "contact_report_bodies": None,
        "unexpected_collision_count": unexpected,
        "initial_penetration_count": penetrations,
        "self_contact_pair_count": self_pairs,
        "pairs": pairs,
    }


def raycast(origin: list[float], direction: list[float], distance: float) -> dict[str, object]:
    hit = get_physx_scene_query_interface().raycast_closest(
        carb.Float3(*origin),
        carb.Float3(*direction),
        float(distance),
        False,
    )
    if not hit or "collision" not in hit:
        return {"hit": False, "origin_m": origin, "direction": direction}
    return {
        "hit": True,
        "origin_m": origin,
        "direction": direction,
        "distance_m": float(hit["distance"]),
        "collision": str(hit["collision"]),
        "position_m": [float(hit["position"][0]), float(hit["position"][1]), float(hit["position"][2])],
    }


def clearance_rays() -> dict[str, object]:
    rays = []
    robot_distances = []
    for target in RESOLVED["bolt_targets_world"]:
        origin = [
            target["position_world_m"][0] - 0.005,
            target["position_world_m"][1],
            target["position_world_m"][2],
        ]
        hit = raycast(origin, [-1.0, 0.0, 0.0], 2.0)
        hit["bolt_id"] = target["id"]
        rays.append(hit)
        if hit["hit"] and "/ur10e" in hit["collision"]:
            robot_distances.append(hit["distance_m"])
    panel_probe = raycast([0.5, 0.0, 1.2], [1.0, 0.0, 0.0], 2.0)
    return {
        "api": "omni.physx.get_physx_scene_query_interface().raycast_closest",
        "note": "Bolt rays start 5 mm on the robot side of the panel face and point -X. A hit on /ur10e is directional clearance, not a mesh signed distance.",
        "bolt_rays": rays,
        "panel_probe": panel_probe,
        "minimum_robot_clearance_m": min(robot_distances) if robot_distances else None,
        "panel_collision_query_hit": bool(panel_probe["hit"] and "BoltPanel" in panel_probe.get("collision", "")),
    }


def version_text() -> str:
    path = ISAAC_ROOT / "VERSION"
    return path.read_text(encoding="utf-8").strip() if path.exists() else ""


def pose_delta(left: dict, right: dict) -> float:
    return distance_m(position_m(left), position_m(right))


def main() -> int:
    SimulationManager.set_physics_sim_device("cpu")
    first = build_once()
    second = build_once()
    build_difference = max_position_difference_m(first["positions"], second["positions"])
    bolt_names = [target["id"] for target in CONFIG["bolt_targets"]]
    bolt_difference = max_position_difference_m(
        {name: first["positions"][name] for name in bolt_names},
        {name: second["positions"][name] for name in bolt_names},
    )
    stage = second["stage"]
    paths = second["paths"]
    expected_base = CONFIG["robot"]["T_world_robot_base"]["xyz_m"]
    base_authoring_error = distance_m(second["positions"]["robot_base"], expected_base)
    tool_path = paths["wrist_3_link"] + "/" + CONFIG["tool_placeholder"]["collision_name"]
    tool_world = position_m(xform_pose(stage, tool_path))
    expected_tool = RESOLVED["tool_box_world_aabb_m"]
    expected_tool_center = [
        (expected_tool["min_m"][axis] + expected_tool["max_m"][axis]) * 0.5 for axis in range(3)
    ]
    tool_collision_error = distance_m(tool_world, expected_tool_center)
    print("tool_collision_world_m " + str(tool_world))
    print("tool_collision_expected_m " + str(expected_tool_center))
    print("tool_collision_authoring_error_m " + str(tool_collision_error))
    print("wrist3_world_m " + str(position_m(xform_pose(stage, paths["wrist_3_link"]))))
    bolt_authoring_error = max(
        distance_m(second["positions"][target["id"]], target["position_world_m"])
        for target in RESOLVED["bolt_targets_world"]
    )
    panel_authoring_error = distance_m(
        second["positions"]["bolt_panel"],
        RESOLVED["T_world_bolt_panel"]["xyz_m"],
    )
    static_authoring_errors = [
        distance_m(position_m(xform_pose(stage, CONFIG["floor"]["prim"])), CONFIG["floor"]["center_m"]),
        distance_m(position_m(xform_pose(stage, CONFIG["fixture"]["prim"])), CONFIG["fixture"]["center_m"]),
    ]
    for post in RESOLVED["posts_world"]:
        static_authoring_errors.append(distance_m(position_m(xform_pose(stage, post["prim"])), post["center_m"]))
    print("static_authoring_error_m " + str(max(static_authoring_errors)))
    root = second["root"]
    robot = Articulation(root)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()
    physics_dt = float(SimulationManager.get_physics_dt())
    dof_indices, joint_names = dof_setup(robot)
    placement_method = "usd_translate_on_/ur10e"
    placed = position_m(xform_pose(stage, paths["base_link"]))
    if distance_m(placed, expected_base) > 1.0e-3:
        robot.set_world_poses(
            positions=np.asarray([expected_base], dtype=np.float64),
            orientations=np.asarray([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
        )
        simulation_app.update()
        placement_method = "set_world_poses"
    regulate(robot, dof_indices, ZERO)
    settled, settle_steps, settle_timeout = settle(robot, dof_indices, physics_dt)
    contact_bodies = enable_contact_reports(stage)
    simulation_app.update()
    try:
        contacts = read_contacts()
    except Exception as exc:
        contacts = {
            "api": "omni.physx.get_physx_simulation_interface().get_contact_report",
            "error": type(exc).__name__ + ": " + str(exc),
            "unexpected_collision_count": None,
            "initial_penetration_count": None,
            "pairs": [],
        }
    contacts["contact_report_bodies"] = contact_bodies
    if contact_bodies == 0:
        contacts["unexpected_collision_count"] = None
        contacts["initial_penetration_count"] = None
        contacts["error"] = "PhysxContactReportAPI was not applied to any rigid body"
    try:
        rays = clearance_rays()
    except Exception as exc:
        rays = {
            "api": "omni.physx.get_physx_scene_query_interface().raycast_closest",
            "error": type(exc).__name__ + ": " + str(exc),
            "panel_collision_query_hit": False,
            "minimum_robot_clearance_m": None,
            "note": "raycast failed",
            "bolt_rays": [],
            "panel_probe": {"hit": False},
        }
    initial = state_snapshot(stage, robot, dof_indices, joint_names, paths)
    duration = float(PHYSICS["stability_duration_s"])
    stability_steps = int(round(duration / physics_dt))
    for _ in range(stability_steps):
        simulation_app.update()
    final = state_snapshot(stage, robot, dof_indices, joint_names, paths)
    timeline.stop()
    simulation_app.update()
    robot = Articulation(root)
    timeline.play()
    simulation_app.update()
    dof_indices, joint_names = dof_setup(robot)
    regulate(robot, dof_indices, ZERO)
    _, reset_steps, reset_timeout = settle(robot, dof_indices, physics_dt)
    after_reset = state_snapshot(stage, robot, dof_indices, joint_names, paths)
    timeline.stop()
    simulation_app.update()

    joint_drift = [abs(final["joint_rad"][index] - initial["joint_rad"][index]) for index in range(len(joint_names))]
    reset_joint_drift = [
        abs(after_reset["joint_rad"][index] - initial["joint_rad"][index]) for index in range(len(joint_names))
    ]
    counts = second["collision_counts"]
    collision_geometry = all(value > 0 for value in counts.values())
    required_prims = all(
        stage.GetPrimAtPath(path)
        for path in (
            "/World",
            "/World/RobotBase",
            CONFIG["floor"]["prim"],
            CONFIG["fixture"]["prim"],
            CONFIG["workcell"]["prim"],
            CONFIG["work_structure"]["prim"],
            CONFIG["bolt_panel"]["prim"],
            paths["base_link"],
            paths["tool0"],
            paths["tool0"] + "/" + CONFIG["tool_placeholder"]["frame_name"],
            tool_path,
        )
    )
    required_frames = required_prims
    non_finite = non_finite_count(initial) + non_finite_count(final) + non_finite_count(after_reset)
    measurements = {
        "acceptance": CONFIG["acceptance"],
        "build_position_difference_m": build_difference,
        "base_authoring_error_m": base_authoring_error,
        "tool_collision_authoring_error_m": tool_collision_error,
        "bolt_authoring_error_m": max([bolt_authoring_error, panel_authoring_error, *static_authoring_errors]),
        "initial_base_error_m": distance_m(position_m(initial["robot_base"]), expected_base),
        "required_prims_present": required_prims,
        "required_frames_present": required_frames,
        "collision_geometry_present": collision_geometry,
        "panel_collision_query_hit": rays["panel_collision_query_hit"],
        "finite_transforms": non_finite == 0,
        "non_finite_count": non_finite,
        "base_displacement_m": pose_delta(initial["robot_base"], final["robot_base"]),
        "max_joint_drift_rad": max(joint_drift),
        "tool0_drift_m": pose_delta(initial["tool0"], final["tool0"]),
        "floor_displacement_m": pose_delta(initial["floor"], final["floor"]),
        "work_structure_displacement_m": pose_delta(initial["work_structure"], final["work_structure"]),
        "bolt_panel_displacement_m": pose_delta(initial["bolt_panel"], final["bolt_panel"]),
        "pedestal_displacement_m": pose_delta(initial["pedestal"], final["pedestal"]),
        "unexpected_collision_count": contacts["unexpected_collision_count"],
        "initial_penetration_count": contacts["initial_penetration_count"],
        "reset_robot_base_difference_m": pose_delta(initial["robot_base"], after_reset["robot_base"]),
        "reset_joint_difference_rad": max(reset_joint_drift),
        "reset_tool0_difference_m": pose_delta(initial["tool0"], after_reset["tool0"]),
        "reset_bolt_panel_difference_m": pose_delta(initial["bolt_panel"], after_reset["bolt_panel"]),
        "bolt_targets_build_difference_m": bolt_difference,
    }
    judgement = judge_workcell(measurements)
    gpu = ""
    try:
        gpu = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        gpu = "unavailable: " + type(exc).__name__
    scene_validation = {
        "required_prims_present": required_prims,
        "required_frames_present": required_frames,
        "frames": {
            "world": "/World",
            "robot_base": "/World/RobotBase",
            "workcell": CONFIG["workcell"]["prim"],
            "bolt_panel": CONFIG["bolt_panel"]["prim"],
            "tool0": paths["tool0"],
        },
        "robot_fixed_base": measurements["base_displacement_m"] <= CONFIG["acceptance"]["base_displacement_m_max"],
        "static_environment": max(
            measurements["floor_displacement_m"],
            measurements["work_structure_displacement_m"],
            measurements["bolt_panel_displacement_m"],
            measurements["pedestal_displacement_m"],
        )
        <= CONFIG["acceptance"]["static_displacement_m_max"],
        "collision_geometry_present": collision_geometry,
        "collision_counts": counts,
        "finite_transforms": non_finite == 0,
        "articulation_root_before": second["roots_before"],
        "articulation_root_relocated_count": second["relocated"],
        "articulation_root": root,
        "placement_method": placement_method,
        "self_collision_engine": "NOT IMPLEMENTED. Robot collision prims are present for a later check.",
        "visual_vs_collision": "UR10e visual and collision mesh origins differ in the pinned URDF. Workcell boxes use the same cube for visual and collision. Tool frame is under tool0. Tool collision is parented to wrist_3_link.",
    }
    stability = {
        "duration_s": stability_steps * physics_dt,
        "requested_duration_s": duration,
        "duration_class": PHYSICS["stability_duration_class"],
        "physics_dt_s": physics_dt,
        "physics_steps": stability_steps,
        "initial_settle_steps": settle_steps,
        "initial_settle_timed_out": settle_timeout,
        "initial_settled_joint_rad": settled,
        "initial_robot_base_pose": initial["robot_base"],
        "final_robot_base_pose": final["robot_base"],
        "base_displacement_m": measurements["base_displacement_m"],
        "initial_joint_state": {"names": joint_names, "rad": initial["joint_rad"]},
        "final_joint_state": {"names": joint_names, "rad": final["joint_rad"]},
        "joint_drift_rad": joint_drift,
        "max_joint_drift_rad": measurements["max_joint_drift_rad"],
        "initial_tool0": initial["tool0"],
        "final_tool0": final["tool0"],
        "tool0_drift_m": measurements["tool0_drift_m"],
        "floor_displacement_m": measurements["floor_displacement_m"],
        "work_structure_displacement_m": measurements["work_structure_displacement_m"],
        "bolt_panel_displacement_m": measurements["bolt_panel_displacement_m"],
        "pedestal_displacement_m": measurements["pedestal_displacement_m"],
        "non_finite_count": non_finite,
    }
    reset_results = {
        "method": "timeline.stop, reconstruct Articulation, timeline.play, set canonical joint targets to zero, settle",
        "reset_settle_steps": reset_steps,
        "reset_timed_out": reset_timeout,
        "before_reset": initial,
        "after_reset": after_reset,
        "robot_base_difference_m": measurements["reset_robot_base_difference_m"],
        "joint_difference_rad": reset_joint_drift,
        "max_joint_difference_rad": measurements["reset_joint_difference_rad"],
        "tool0_difference_m": measurements["reset_tool0_difference_m"],
        "bolt_panel_difference_m": measurements["reset_bolt_panel_difference_m"],
    }
    collision_results = {
        **contacts,
        "raycast": rays,
        "minimum_separation_m": rays["minimum_robot_clearance_m"],
        "minimum_separation_note": rays["note"],
        "geometric_distance_robot_base_to_panel_m": distance_m(
            expected_base,
            RESOLVED["T_world_bolt_panel"]["xyz_m"],
        ),
        "geometric_sanity": RESOLVED["geometric_sanity"],
    }
    environment = "\n".join(
        [
            "# P-005 environment",
            "",
            "claim: " + CONFIG["claim"],
            "geometry_class: " + CONFIG["geometry_class"],
            "isaac_version: " + version_text(),
            "execution_mode: headless",
            "physics_device: cpu",
            "os: " + platform.platform(),
            "gpu: " + gpu,
            "physics_dt_s: " + str(physics_dt),
            "gravity: " + str(second["gravity"]),
            "drive_stiffness_nm_per_rad: " + str(IMPORTER["override_joint_stiffness_nm_per_rad"]),
            "drive_damping: " + str(IMPORTER["override_joint_damping"]),
            "articulation_root: " + root,
            "placement_method: " + placement_method,
            "up_axis: Z",
            "handedness: right",
            "tool0_observation_api: " + POSE_API,
            "contact_api: " + contacts["api"],
            "raycast_api: " + rays["api"],
            "self_collision_engine: not implemented",
            "unity_workcell: not duplicated in this gate. Isaac is the physics baseline.",
            "",
        ]
    )
    write_evidence(
        EVIDENCE,
        {
            "workcell_config": {
                **RESOLVED,
                "runtime": {
                    "placement_method": placement_method,
                    "gravity": second["gravity"],
                    "physics_dt_s": physics_dt,
                    "articulation_root": root,
                    "generated_usd_dir": str(USD_DIR),
                },
            },
            "scene_validation": scene_validation,
            "stability_results": stability,
            "reset_results": reset_results,
            "collision_results": collision_results,
            "environment_md": environment,
            "summary_md": render_summary(CONFIG["claim"], judgement, measurements),
            "test_results_txt": render_test_results(judgement),
        },
    )
    print("RESULT " + judgement["result"])
    print("base_displacement_m " + str(measurements["base_displacement_m"]))
    print("max_joint_drift_rad " + str(measurements["max_joint_drift_rad"]))
    print("tool0_drift_m " + str(measurements["tool0_drift_m"]))
    print("unexpected_collision_count " + str(measurements["unexpected_collision_count"]))
    print("initial_penetration_count " + str(measurements["initial_penetration_count"]))
    print("reset_robot_base_difference_m " + str(measurements["reset_robot_base_difference_m"]))
    print("build_position_difference_m " + str(measurements["build_position_difference_m"]))
    print("placement_method " + placement_method)
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
