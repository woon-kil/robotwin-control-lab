"""P-003 Fix Attempt 1 diagnostic. Does not change the joint-control runner.

Traces link poses with the current command sequence, then repeats the current
RigidPrim(tool0) observation. RigidPrim.__init__ applies RigidBodyAPI when the
prim does not already have it. This script records that effect; it does not
remove it from the runner.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--repo", required=True)
parser.add_argument("--isaac-root", default=str(Path.home() / "isaacsim"))
args = parser.parse_args()

REPO = Path(args.repo).resolve()
sys.path.insert(0, str(REPO))

from isaac.ur10e_commands import (  # noqa: E402
    CANONICAL_JOINT_ORDER,
    CONFIGURATIONS,
    SOURCE_COMMIT,
    map_canonical_dofs,
    validate_joint_targets,
)

sys.argv = [sys.argv[0]]

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.core.experimental.prims import Articulation, RigidPrim, XformPrim  # noqa: E402
from isaacsim.core.experimental.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

URDF_RELATIVE = Path("robot_description/generated") / SOURCE_COMMIT / "ur10e.urdf"
PACKAGE_RELATIVE = Path("robot_description/upstream") / SOURCE_COMMIT
OUT_DIR = REPO / "evidence" / "P-003" / "fix-attempt1"
USD_DIR = REPO / "isaac" / "generated" / "ur10e"
DRIVE_STIFFNESS = 100000.0
DRIVE_DAMPING = 1000.0
TIMEOUT_SECONDS = 8.0
SETTLED_STEPS = 20
VELOCITY_EPSILON = 0.001
POSITION_DELTA_EPSILON = 1e-5
LINK_NAMES = (
    "world",
    "base",
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
RIGID_PAIRS = (
    ("base_link_inertia", "shoulder_link"),
    ("shoulder_link", "upper_arm_link"),
    ("upper_arm_link", "forearm_link"),
    ("forearm_link", "wrist_1_link"),
    ("wrist_1_link", "wrist_2_link"),
    ("wrist_2_link", "wrist_3_link"),
    ("wrist_3_link", "flange"),
    ("flange", "tool0"),
)


def numbers(value) -> list[float]:
    array = value.numpy() if hasattr(value, "numpy") else np.asarray(value)
    return [float(item) for item in np.reshape(array, -1)]


def xyz_dict(values: list[float]) -> dict[str, float]:
    return {"x": values[0], "y": values[1], "z": values[2]}


def quat_dict(values: list[float]) -> dict[str, float]:
    return {"w": values[0], "x": values[1], "y": values[2], "z": values[3]}


def pose_from_xyz_quat(xyz: list[float], quat: list[float], api: str) -> dict[str, object]:
    return {
        "frame": "isaac_world",
        "quaternion_order": "wxyz",
        "pose_api": api,
        "position_m": xyz_dict(xyz),
        "quaternion_wxyz": quat_dict(quat),
    }


def xform_pose(stage, path: str) -> dict[str, object]:
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    transform = Gf.Transform()
    transform.SetMatrix(matrix)
    translation = transform.GetTranslation()
    quat = transform.GetRotation().GetQuat()
    imaginary = quat.GetImaginary()
    return pose_from_xyz_quat(
        [float(translation[0]), float(translation[1]), float(translation[2])],
        [float(quat.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2])],
        "UsdGeom.XformCache.GetLocalToWorldTransform",
    )


def local_pose(prim) -> dict[str, object]:
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    matrix, _resets = cache.GetLocalTransformation(prim)
    transform = Gf.Transform()
    transform.SetMatrix(matrix)
    translation = transform.GetTranslation()
    quat = transform.GetRotation().GetQuat()
    imaginary = quat.GetImaginary()
    return {
        "translate_m": [float(translation[0]), float(translation[1]), float(translation[2])],
        "quaternion_wxyz": [
            float(quat.GetReal()),
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
        ],
    }


def select_prim(stage, name: str):
    matches = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() == name and str(prim.GetPath()).startswith("/ur10e/")
    ]
    geometry = [prim for prim in matches if "/Geometry/" in str(prim.GetPath())]
    chosen = geometry or matches
    if len(chosen) != 1:
        return None, [str(prim.GetPath()) for prim in matches]
    return chosen[0], [str(prim.GetPath()) for prim in matches]


def identity(stage, prim) -> dict[str, object]:
    return {
        "path": str(prim.GetPath()),
        "type_name": prim.GetTypeName(),
        "parent": str(prim.GetParent().GetPath()),
        "api_schemas": sorted(prim.GetAppliedSchemas()),
        "has_rigid_body": prim.HasAPI(UsdPhysics.RigidBodyAPI),
        "has_articulation_root": prim.HasAPI(UsdPhysics.ArticulationRootAPI),
        "local_pose": local_pose(prim),
    }


def command_targets(robot, dof_indices: list[int], targets: tuple[float, ...]) -> None:
    checked = validate_joint_targets(targets)
    robot.set_dof_position_targets(np.asarray(checked, dtype=np.float64).reshape(1, -1), dof_indices=dof_indices)
    robot.set_dof_velocity_targets(np.zeros((1, len(checked)), dtype=np.float64), dof_indices=dof_indices)


def regulate_from_command(robot, dof_indices: list[int], targets: tuple[float, ...]) -> None:
    checked = validate_joint_targets(targets)
    values = np.asarray(checked, dtype=np.float64).reshape(1, -1)
    zeros = np.zeros_like(values)
    robot.set_dof_velocities(zeros, dof_indices=dof_indices)
    robot.set_dof_positions(values, dof_indices=dof_indices)
    command_targets(robot, dof_indices, targets)


def step_until_settled(robot, dof_indices: list[int], dt: float):
    previous = numbers(robot.get_dof_positions(dof_indices=dof_indices))
    stable = 0
    steps = 0
    elapsed = 0.0
    actual = previous
    velocity = numbers(robot.get_dof_velocities(dof_indices=dof_indices))
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
            return actual, velocity, steps, False
    return actual, velocity, steps, True


def rigid_pose(view) -> dict[str, object]:
    position, orientation = view.get_world_poses()
    pose = pose_from_xyz_quat(numbers(position), numbers(orientation), "RigidPrim.get_world_poses")
    pose["prim_path"] = view.paths[0] if hasattr(view, "paths") else ""
    return pose


def xform_prim_pose(view) -> dict[str, object]:
    position, orientation = view.get_world_poses()
    pose = pose_from_xyz_quat(numbers(position), numbers(orientation), "XformPrim.get_world_poses")
    pose["prim_path"] = view.paths[0] if hasattr(view, "paths") else ""
    return pose


def build_views(links: dict[str, str], rigid_names: set[str], include_tool0_rigid: bool):
    """Create physics views before the articulation view.

    RigidPrim.__init__ flushes the simulation and invalidates an existing
    articulation tensor view. Articulation is therefore constructed last.
    """
    xform_views = {name: XformPrim(path) for name, path in links.items()}
    rigid_views = {name: RigidPrim(path) for name, path in links.items() if name in rigid_names}
    if include_tool0_rigid:
        rigid_views["tool0"] = RigidPrim(links["tool0"])
    robot = Articulation(links["base_link_inertia"])
    return robot, xform_views, rigid_views


def distance(left: dict[str, object], right: dict[str, object]) -> float:
    a = left["position_m"]
    b = right["position_m"]
    return math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"]))


def root_velocity(robot) -> dict[str, object]:
    try:
        linear, angular = robot.get_velocities()
        return {
            "readable": True,
            "linear_m_s": numbers(linear),
            "angular_rad_s": numbers(angular),
        }
    except Exception as exc:  # noqa: BLE001
        return {"readable": False, "error": type(exc).__name__ + ": " + str(exc)}


def observe(stage, robot, links, xform_views, rigid_views) -> dict[str, object]:
    observed = {}
    for name, path in links.items():
        entry = {"path": path, "usd_xform": xform_pose(stage, path)}
        try:
            entry["xform_prim"] = xform_prim_pose(xform_views[name])
        except Exception as exc:  # noqa: BLE001
            entry["xform_prim_error"] = type(exc).__name__ + ": " + str(exc)
        if name in rigid_views:
            try:
                entry["rigid_prim"] = rigid_pose(rigid_views[name])
            except Exception as exc:  # noqa: BLE001
                entry["rigid_prim_error"] = type(exc).__name__ + ": " + str(exc)
        observed[name] = entry
    root_position, root_orientation = robot.get_world_poses()
    return {
        "links": observed,
        "articulation_root_pose": pose_from_xyz_quat(
            numbers(root_position),
            numbers(root_orientation),
            "Articulation.get_world_poses",
        ),
        "articulation_root_velocity": root_velocity(robot),
    }


def pair_distances(snapshot: dict[str, object], source: str) -> dict[str, float | None]:
    found = {}
    for parent, child in RIGID_PAIRS:
        parent_pose = snapshot["links"][parent].get(source)
        child_pose = snapshot["links"][child].get(source)
        if parent_pose is None or child_pose is None:
            found[parent + "->" + child] = None
        else:
            found[parent + "->" + child] = distance(parent_pose, child_pose)
    return found


def run_sequence(stage, robot, xform_views, rigid_views, dof_indices, dt, links, fresh: bool, timeline):
    rows = []
    for test_id, joints in CONFIGURATIONS:
        sequence = ["set_dof_velocities(0)", "set_dof_positions(command)", "set_dof_position_targets(command)"]
        if fresh:
            timeline.stop()
            simulation_app.update()
            robot = Articulation(links["base_link_inertia"])
            timeline.play()
            simulation_app.update()
            xform_views = {name: XformPrim(path) for name, path in links.items()}
            sequence = [
                "timeline.stop",
                "simulation_app.update",
                "Articulation(base_link_inertia) before play",
                "timeline.play",
                "simulation_app.update",
                "XformPrim views, no RigidPrim",
                *sequence,
            ]
        regulate_from_command(robot, dof_indices, joints)
        simulation_app.update()
        actual, velocity, steps, timed_out = step_until_settled(robot, dof_indices, dt)
        snapshot = observe(stage, robot, links, xform_views, rigid_views)
        tool0_now = stage.GetPrimAtPath(links["tool0"])
        rows.append(
            {
                "test_id": test_id,
                "initialization_sequence": sequence,
                "simulation_time_s": steps * dt,
                "settling_steps": steps,
                "timed_out": timed_out,
                "target_joint_rad": list(joints),
                "actual_joint_rad": actual,
                "joint_velocity_rad_s": velocity,
                "tool0_has_rigid_body_after_observe": tool0_now.HasAPI(UsdPhysics.RigidBodyAPI),
                "snapshot": snapshot,
                "usd_pair_distance_m": pair_distances(snapshot, "usd_xform"),
                "rigid_pair_distance_m": pair_distances(snapshot, "rigid_prim"),
            }
        )
    return rows


def z_values(rows: list[dict[str, object]], name: str, source: str) -> list[float | None]:
    values = []
    for row in rows:
        pose = row["snapshot"]["links"][name].get(source)
        values.append(None if pose is None else pose["position_m"]["z"])
    return values


def free_fall_after_rigid_prim(stage, path: str) -> list[dict[str, object]]:
    """Apply RigidPrim to tool0, then step physics without a new joint command.

    This isolates the observer from the joint controller. Samples match the
    runner's 43 updates per configuration (one update plus 42 settled steps).
    """
    view = RigidPrim(path)
    rows = []
    for sample in range(5):
        steps = 0 if sample == 0 else 43
        for _ in range(steps):
            simulation_app.update()
        position, orientation = view.get_world_poses()
        pose = pose_from_xyz_quat(numbers(position), numbers(orientation), "RigidPrim.get_world_poses")
        pose["prim_path"] = path
        snapshot = {
            "links": {
                "tool0": {"rigid_prim": pose, "usd_xform": xform_pose(stage, path)},
                "wrist_3_link": {},
                "base_link_inertia": {},
            }
        }
        rows.append(
            {
                "test_id": "free-fall-" + str(sample),
                "initialization_sequence": [
                    "RigidPrim(tool0), which applies RigidBodyAPI when it is absent",
                    "simulation_app.update x " + str(steps),
                ],
                "simulation_time_s": steps * (1.0 / 60.0),
                "settling_steps": steps,
                "timed_out": False,
                "snapshot": snapshot,
                "tool0_has_rigid_body_after_observe": stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI),
            }
        )
    return rows


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
    links = {}
    ambiguous = {}
    identities = {}
    for name in LINK_NAMES:
        prim, matches = select_prim(stage, name)
        if prim is None:
            ambiguous[name] = matches
            continue
        links[name] = str(prim.GetPath())
        identities[name] = identity(stage, prim)
    fixed_joints = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.FixedJoint):
            continue
        joint = UsdPhysics.Joint(prim)
        fixed_joints.append(
            {
                "path": str(prim.GetPath()),
                "body0": str(joint.GetBody0Rel().GetTargets()),
                "body1": str(joint.GetBody1Rel().GetTargets()),
            }
        )
    robot = Articulation(links["base_link_inertia"])
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()
    rigid_names = {name for name, info in identities.items() if info["has_rigid_body"]}
    dof_names = list(robot.dof_names)
    if "shoulder_pan_joint" not in dof_names:
        raise RuntimeError("dof_names=" + repr(dof_names))
    mapping = map_canonical_dofs(dof_names)
    dof_indices = []
    for row in mapping:
        api_index = int(numbers(robot.get_dof_indices(str(row["isaac_dof_name"])))[0])
        dof_indices.append(api_index)
    dt = float(SimulationManager.get_physics_dt())
    tool0_schemas_before = list(identities["tool0"]["api_schemas"])
    xform_views = {name: XformPrim(path) for name, path in links.items()}
    rigid_views = {}
    mode_b = run_sequence(stage, robot, xform_views, rigid_views, dof_indices, dt, links, False, timeline)
    mode_b_fresh = run_sequence(stage, robot, xform_views, rigid_views, dof_indices, dt, links, True, timeline)
    mode_a = free_fall_after_rigid_prim(stage, links["tool0"])
    report = {
        "gate": "P-003",
        "diagnostic": "fix-attempt1",
        "generated_asset": generated,
        "source_commit": SOURCE_COMMIT,
        "physics_dt_s": dt,
        "canonical_joint_order": list(CANONICAL_JOINT_ORDER),
        "ambiguous_prims": ambiguous,
        "link_identity": identities,
        "fixed_joints": fixed_joints,
        "tool0_api_schemas_before_rigid_prim": tool0_schemas_before,
        "modes": {
            "mode_b_continuous": {
                "description": "one simulation, current teleport between configurations, no RigidPrim(tool0)",
                "configurations": mode_b,
            },
            "mode_b_fresh": {
                "description": "timeline.stop, new Articulation, timeline.play before each configuration, no RigidPrim(tool0)",
                "configurations": mode_b_fresh,
            },
            "mode_a_current_reader": {
                "description": "current runner observation: RigidPrim(tool0) after settle. Constructor applies RigidBodyAPI if absent.",
                "configurations": mode_a,
            },
        },
        "z_trace": {
            "mode_b_tool0_usd": z_values(mode_b, "tool0", "usd_xform"),
            "mode_b_wrist_3_rigid": z_values(mode_b, "wrist_3_link", "rigid_prim"),
            "mode_b_fresh_tool0_usd": z_values(mode_b_fresh, "tool0", "usd_xform"),
            "mode_a_tool0_rigid": z_values(mode_a, "tool0", "rigid_prim"),
            "mode_a_tool0_usd": z_values(mode_a, "tool0", "usd_xform"),
            "mode_a_wrist_3_rigid": z_values(mode_a, "wrist_3_link", "rigid_prim"),
            "mode_a_base_link_inertia_rigid": z_values(mode_a, "base_link_inertia", "rigid_prim"),
        },
    }
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "state_trace.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("WROTE", OUT_DIR / "state_trace.json")
    print("Z", json.dumps(report["z_trace"]))
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
