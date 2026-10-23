"""Isaac link chain을 같은 actual joint의 Python FK와 비교한다.

evidence/P-004 루트는 쓰지 않는다. RigidPrim은 만들지 않는다.
drive, timestep, URDF는 P-004 runner와 같다.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--repo", required=True)
parser.add_argument("--isaac-root", default=str(Path.home() / "isaacsim"))
parser.add_argument("--relocate-articulation-root", action="store_true")
args = parser.parse_args()

REPO = Path(args.repo).resolve()
sys.path.insert(0, str(REPO))

from isaac.ur10e_commands import CONFIGURATIONS, SOURCE_COMMIT, map_canonical_dofs, validate_joint_targets  # noqa: E402

sys.argv = [sys.argv[0]]

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.experimental.utils import stage as stage_utils  # noqa: E402
from isaacsim.asset.importer.utils.impl.asset_utils import fix_articulation_root_for_fixed_base  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

from core.kinematics.fk import chain_frames  # noqa: E402
from core.model.ur10e import load_ur10e_model  # noqa: E402
from core.validation.tcp_compare import (  # noqa: E402
    invert_rigid,
    isaac_pose_to_canonical,
    matrix_to_pose,
    multiply,
    orientation_error_rad,
    position_error_m,
)

URDF_RELATIVE = Path("robot_description/generated") / SOURCE_COMMIT / "ur10e.urdf"
PACKAGE_RELATIVE = Path("robot_description/upstream") / SOURCE_COMMIT
EVIDENCE = REPO / "evidence" / "P-004" / "fix-attempt1"
USD_DIR = REPO / "isaac" / "generated" / "ur10e"
DRIVE_STIFFNESS = 100000.0
DRIVE_DAMPING = 1000.0
TIMEOUT_SECONDS = 8.0
SETTLED_STEPS = 20
VELOCITY_EPSILON = 0.001
POSITION_DELTA_EPSILON = 1e-5
CHAIN = (
    "base_link",
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "flange",
    "tool0",
)


def numbers(value) -> list[float]:
    array = value.numpy() if hasattr(value, "numpy") else np.asarray(value)
    return [float(item) for item in np.reshape(array, -1)]


def precise(values: list[float]) -> list[str]:
    return [format(float(value), ".17g") for value in values]


def unique_geometry_prim(stage, name: str) -> str:
    matches = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.GetName() == name and str(prim.GetPath()).startswith("/ur10e/Geometry/")
    ]
    if len(matches) != 1:
        raise RuntimeError("expected one " + name + ", found " + str(matches))
    return matches[0]


def matrix_pose(matrix) -> dict[str, float]:
    transform = Gf.Transform()
    transform.SetMatrix(matrix)
    translation = transform.GetTranslation()
    quat = transform.GetRotation().GetQuat()
    imaginary = quat.GetImaginary()
    return {
        "x": float(translation[0]),
        "y": float(translation[1]),
        "z": float(translation[2]),
        "qw": float(quat.GetReal()),
        "qx": float(imaginary[0]),
        "qy": float(imaginary[1]),
        "qz": float(imaginary[2]),
    }


def world_pose(stage, path: str) -> dict[str, object]:
    prim = stage.GetPrimAtPath(path)
    if prim.GetName() == "tool0" and prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError("tool0 gained RigidBodyAPI")
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    pose = matrix_pose(cache.GetLocalToWorldTransform(prim))
    pose["prim_path"] = path
    pose["rigid_body_api"] = bool(prim.HasAPI(UsdPhysics.RigidBodyAPI))
    pose["type_name"] = prim.GetTypeName()
    return pose


def local_pose(stage, path: str) -> dict[str, object]:
    prim = stage.GetPrimAtPath(path)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    matrix, _resets = cache.GetLocalTransformation(prim)
    pose = matrix_pose(matrix)
    pose["prim_path"] = path
    pose["parent"] = str(prim.GetParent().GetPath())
    return pose


def child_notes(stage, path: str) -> list[dict[str, object]]:
    notes = []
    for child in stage.GetPrimAtPath(path).GetChildren():
        name = child.GetName()
        if name in {"visual", "collision"} or "visual" in name or "collision" in name:
            kind = "visual_or_collision"
        elif "inertia" in name or "inertial" in name:
            kind = "inertial"
        else:
            kind = "other"
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        matrix, _resets = cache.GetLocalTransformation(child)
        pose = matrix_pose(matrix)
        notes.append(
            {
                "name": name,
                "type_name": child.GetTypeName(),
                "kind": kind,
                "rigid_body_api": bool(child.HasAPI(UsdPhysics.RigidBodyAPI)),
                "local_translation_m": [pose["x"], pose["y"], pose["z"]],
            }
        )
    return notes


def usd_component(value) -> object:
    if value is None:
        return None
    if hasattr(value, "GetImaginary") and hasattr(value, "GetReal"):
        imaginary = value.GetImaginary()
        return {
            "w": float(value.GetReal()),
            "x": float(imaginary[0]),
            "y": float(imaginary[1]),
            "z": float(imaginary[2]),
        }
    try:
        return [float(item) for item in value]
    except TypeError:
        return str(value)


def physics_joints(stage) -> list[dict[str, object]]:
    rows = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        body0 = [str(path) for path in joint.GetBody0Rel().GetTargets()]
        body1 = [str(path) for path in joint.GetBody1Rel().GetTargets()]
        axis = None
        axis_attr = prim.GetAttribute("physics:axis")
        if axis_attr.IsValid() and axis_attr.Get() is not None:
            axis = str(axis_attr.Get())
        rows.append(
            {
                "path": str(prim.GetPath()),
                "type_name": prim.GetTypeName(),
                "body0": body0,
                "body1": body1,
                "local_pos0": usd_component(joint.GetLocalPos0Attr().Get()),
                "local_rot0": usd_component(joint.GetLocalRot0Attr().Get()),
                "local_pos1": usd_component(joint.GetLocalPos1Attr().Get()),
                "local_rot1": usd_component(joint.GetLocalRot1Attr().Get()),
                "axis": axis,
            }
        )
    return rows


def python_chain(joints: list[float]) -> dict[str, dict[str, object]]:
    robot = load_ur10e_model()
    frames = {"base_link": {"cumulative": {"frame": "base_link", "reference_frame": "base_link", "x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}}}
    for frame in chain_frames(robot, joints):
        frames[str(frame["child"])] = {
            "joint": frame["joint"],
            "joint_type": frame["joint_type"],
            "parent": frame["parent"],
            "origin_xyz_m": frame["origin_xyz_m"],
            "origin_rpy_rad": frame["origin_rpy_rad"],
            "axis_xyz": frame["axis_xyz"],
            "local": frame["local"],
            "cumulative": frame["cumulative"],
        }
    return frames


def relative_pose(world_base: dict[str, float], world_link: dict[str, float]) -> dict[str, float | str]:
    base = isaac_pose_to_canonical(
        world_base,
        {"w": world_base["qw"], "x": world_base["qx"], "y": world_base["qy"], "z": world_base["qz"]},
    )
    link = isaac_pose_to_canonical(
        world_link,
        {"w": world_link["qw"], "x": world_link["qx"], "y": world_link["qy"], "z": world_link["qz"]},
    )
    return matrix_to_pose(multiply(invert_rigid(base), link), str(world_link.get("frame", "link")), "base_link")


def pose_delta(reference: dict, test: dict) -> dict[str, float]:
    ref = {"x": float(reference["x"]), "y": float(reference["y"]), "z": float(reference["z"]), "qx": float(reference["qx"]), "qy": float(reference["qy"]), "qz": float(reference["qz"]), "qw": float(reference["qw"])}
    obs = {"x": float(test["x"]), "y": float(test["y"]), "z": float(test["z"]), "qx": float(test["qx"]), "qy": float(test["qy"]), "qz": float(test["qz"]), "qw": float(test["qw"])}
    position_m = position_error_m(ref, obs)
    orientation_rad = orientation_error_rad(
        {"x": ref["qx"], "y": ref["qy"], "z": ref["qz"], "w": ref["qw"]},
        {"x": obs["qx"], "y": obs["qy"], "z": obs["qz"], "w": obs["qw"]},
    )
    return {
        "dx_m": obs["x"] - ref["x"],
        "dy_m": obs["y"] - ref["y"],
        "dz_m": obs["z"] - ref["z"],
        "dx_mm": (obs["x"] - ref["x"]) * 1000.0,
        "dy_mm": (obs["y"] - ref["y"]) * 1000.0,
        "dz_mm": (obs["z"] - ref["z"]) * 1000.0,
        "position_error_m": position_m,
        "position_error_mm": position_m * 1000.0,
        "orientation_error_rad": orientation_rad,
        "orientation_error_deg": orientation_rad * 180.0 / 3.141592653589793,
    }


def compare_chain(python_frames: dict[str, dict], isaac_base_poses: dict[str, dict]) -> list[dict[str, object]]:
    rows = []
    previous = None
    for name in CHAIN:
        python_cumulative = python_frames[name]["cumulative"]
        isaac_cumulative = relative_pose(isaac_base_poses["base_link"], isaac_base_poses[name])
        cumulative_error = pose_delta(python_cumulative, isaac_cumulative)
        local_error = None
        if previous is not None and "local" in python_frames[name]:
            parent_pose = relative_pose(isaac_base_poses["base_link"], isaac_base_poses[previous])
            child_pose = isaac_cumulative
            isaac_local = matrix_to_pose(
                multiply(
                    invert_rigid(
                        isaac_pose_to_canonical(
                            parent_pose,
                            {"w": parent_pose["qw"], "x": parent_pose["qx"], "y": parent_pose["qy"], "z": parent_pose["qz"]},
                        )
                    ),
                    isaac_pose_to_canonical(
                        child_pose,
                        {"w": child_pose["qw"], "x": child_pose["qx"], "y": child_pose["qy"], "z": child_pose["qz"]},
                    ),
                ),
                name,
                previous,
            )
            local_error = pose_delta(python_frames[name]["local"], isaac_local)
        rows.append(
            {
                "frame": name,
                "joint": python_frames[name].get("joint"),
                "parent": python_frames[name].get("parent"),
                "python_cumulative": python_cumulative,
                "isaac_cumulative_base": isaac_cumulative,
                "cumulative_error": cumulative_error,
                "local_error": local_error,
                "prim_path": isaac_base_poses[name]["prim_path"],
                "rigid_body_api": isaac_base_poses[name]["rigid_body_api"],
            }
        )
        previous = name
    return rows


def first_divergence(rows: list[dict[str, object]]) -> dict[str, object] | None:
    for row in rows:
        error = row["cumulative_error"]
        if float(error["position_error_mm"]) >= 0.05 or float(error["orientation_error_deg"]) >= 0.01:
            return {"frame": row["frame"], "joint": row["joint"], "position_error_mm": error["position_error_mm"], "orientation_error_deg": error["orientation_error_deg"]}
    return None


def regulate(robot, dof_indices: list[int], targets: list[float]) -> None:
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


def read_chain(stage, paths: dict[str, str]) -> dict[str, dict]:
    return {name: world_pose(stage, path) for name, path in paths.items()}


def fresh(timeline, root: str):
    timeline.stop()
    simulation_app.update()
    robot = Articulation(root)
    timeline.play()
    simulation_app.update()
    return robot


def dof_indices_for(robot) -> list[int]:
    mapping = map_canonical_dofs(list(robot.dof_names))
    return [int(numbers(robot.get_dof_indices(str(row["isaac_dof_name"])))[0]) for row in mapping]


def main() -> int:
    urdf_path = REPO / URDF_RELATIVE
    package_root = REPO / PACKAGE_RELATIVE
    SimulationManager.set_physics_sim_device("cpu")
    USD_DIR.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(urdf_path),
        usd_path=str(USD_DIR),
        merge_fixed_joints=False,
        merge_mesh=False,
        collision_from_visuals=False,
        allow_self_collision=False,
        ros_package_paths=[{"name": "ur_description", "path": str(package_root)}],
        fix_base=True,
        joint_drive_type="force",
        joint_target_type="position",
        override_joint_stiffness=DRIVE_STIFFNESS,
        override_joint_damping=DRIVE_DAMPING,
        run_asset_transformer=False,
        run_multi_physics_conversion=True,
    )
    generated = URDFImporter(config).import_urdf()
    opened, stage = stage_utils.open_stage(generated)
    if not opened or stage is None:
        raise RuntimeError("failed to open generated USD " + generated)
    if not any(prim.IsA(UsdPhysics.Scene) for prim in stage.Traverse()):
        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
    roots_before = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    relocated = 0
    if args.relocate_articulation_root:
        relocated = int(fix_articulation_root_for_fixed_base(stage))
    roots = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError("expected one articulation root, found " + str(roots))
    paths = {name: unique_geometry_prim(stage, name) for name in CHAIN}
    static_local = {name: local_pose(stage, path) for name, path in paths.items()}
    static_children = {name: child_notes(stage, path) for name, path in paths.items()}
    joints = physics_joints(stage)
    zero_python = python_chain([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    origin_rows = []
    for name in CHAIN:
        if name not in zero_python or "local" not in zero_python[name]:
            continue
        origin_rows.append(
            {
                "frame": name,
                "joint": zero_python[name].get("joint"),
                "parent": zero_python[name].get("parent"),
                "urdf_origin_xyz_m": zero_python[name]["origin_xyz_m"],
                "urdf_origin_rpy_rad": zero_python[name]["origin_rpy_rad"],
                "python_local_at_zero": zero_python[name]["local"],
                "isaac_authored_local": static_local[name],
                "authored_versus_python_zero": pose_delta(zero_python[name]["local"], _xyzw_pose(static_local[name])),
                "children": static_children[name],
            }
        )
    timeline = omni.timeline.get_timeline_interface()
    robot = Articulation(roots[0])
    timeline.play()
    simulation_app.update()
    physics_dt = float(SimulationManager.get_physics_dt())
    configurations = []
    for test_id, target in CONFIGURATIONS:
        robot = fresh(timeline, roots[0])
        indices = dof_indices_for(robot)
        regulate(robot, indices, list(target))
        simulation_app.update()
        pre_joints = numbers(robot.get_dof_positions(dof_indices=indices))
        pre_chain = compare_chain(python_chain(pre_joints), read_chain(stage, paths))
        post_joints, steps, timed_out = settle(robot, indices, physics_dt)
        post_chain = compare_chain(python_chain(post_joints), read_chain(stage, paths))
        robot = fresh(timeline, roots[0])
        indices = dof_indices_for(robot)
        regulate(robot, indices, post_joints)
        simulation_app.update()
        injected_joints = numbers(robot.get_dof_positions(dof_indices=indices))
        injected_chain = compare_chain(python_chain(injected_joints), read_chain(stage, paths))
        configurations.append(
            {
                "test_id": test_id,
                "target_joint_rad": precise(list(target)),
                "pre_step_joint_rad": precise(pre_joints),
                "post_settle_joint_rad": precise(post_joints),
                "injected_joint_rad": precise(injected_joints),
                "settling_steps": steps,
                "timed_out": timed_out,
                "pre_step_first_divergence": first_divergence(pre_chain),
                "post_settle_first_divergence": first_divergence(post_chain),
                "injected_first_divergence": first_divergence(injected_chain),
                "pre_step": [_public_row(row) for row in pre_chain],
                "post_settle": [_public_row(row) for row in post_chain],
                "injected_settled_joints": [_public_row(row) for row in injected_chain],
            }
        )
    payload = {
        "gate": "P-004",
        "diagnostic": "fix-attempt1",
        "source_commit": SOURCE_COMMIT,
        "pose_api": "UsdGeom.XformCache.GetLocalToWorldTransform",
        "rigid_prim_constructed": False,
        "physics_dt_s": physics_dt,
        "articulation_root_before": roots_before,
        "articulation_root": roots[0],
        "articulation_root_relocated_count": relocated,
        "configurations": configurations,
    }
    origins = {
        "gate": "P-004",
        "diagnostic": "fix-attempt1",
        "source_commit": SOURCE_COMMIT,
        "frames": origin_rows,
        "physics_joints": joints,
    }
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    link_name = "link_comparison_relocated.json" if args.relocate_articulation_root else "link_comparison.json"
    (EVIDENCE / link_name).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if not args.relocate_articulation_root:
        (EVIDENCE / "joint_origin_comparison.json").write_text(json.dumps(origins, indent=2) + "\n", encoding="utf-8")
    simulation_app.close()
    print("WROTE", EVIDENCE)
    for row in configurations:
        post = row["post_settle_first_divergence"]
        pre = row["pre_step_first_divergence"]
        injected = row["injected_first_divergence"]
        tool = row["post_settle"][-1]["cumulative_error"]
        print(row["test_id"], "pre", pre, "post", post, "injected", injected, "tool_mm", tool["dx_mm"], tool["dy_mm"], tool["dz_mm"])
    return 0


def _xyzw_pose(pose: dict) -> dict[str, float]:
    return {"x": pose["x"], "y": pose["y"], "z": pose["z"], "qx": pose["qx"], "qy": pose["qy"], "qz": pose["qz"], "qw": pose["qw"]}


def _public_row(row: dict[str, object]) -> dict[str, object]:
    return {
        "frame": row["frame"],
        "joint": row["joint"],
        "parent": row["parent"],
        "prim_path": row["prim_path"],
        "rigid_body_api": row["rigid_body_api"],
        "cumulative_error": row["cumulative_error"],
        "local_error": row["local_error"],
    }


if __name__ == "__main__":
    raise SystemExit(main())
