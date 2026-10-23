"""Headless Isaac UR10e tool0 observation for P-004.

Writes evidence/P-004/raw/isaac_results.json only. Does not construct RigidPrim.
"""

from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--repo", required=True)
parser.add_argument("--isaac-root", default=str(Path.home() / "isaacsim"))
args = parser.parse_args()

REPO = Path(args.repo).resolve()
ISAAC_ROOT = Path(args.isaac_root).resolve()
sys.path.insert(0, str(REPO))

from isaac.ur10e_commands import (  # noqa: E402
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
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.experimental.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

URDF_RELATIVE = Path("robot_description/generated") / SOURCE_COMMIT / "ur10e.urdf"
PACKAGE_RELATIVE = Path("robot_description/upstream") / SOURCE_COMMIT
EVIDENCE = REPO / "evidence" / "P-004" / "raw"
USD_DIR = REPO / "isaac" / "generated" / "ur10e"
DRIVE_STIFFNESS = 100000.0
DRIVE_DAMPING = 1000.0
TIMEOUT_SECONDS = 8.0
SETTLED_STEPS = 20
VELOCITY_EPSILON = 0.001
POSITION_DELTA_EPSILON = 1e-5


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
        "pose_api": "UsdGeom.XformCache.GetLocalToWorldTransform Gf.Quatd real then imaginary",
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


def ensure_gravity(stage) -> None:
    scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if scenes:
        return
    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)


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


def measure(stage, robot, dof_indices: list[int], dt: float, test_id: str, joints: tuple[float, ...], base_path: str, tool_path: str) -> dict[str, object]:
    regulate(robot, dof_indices, joints)
    simulation_app.update()
    actual, steps, timed_out = settle(robot, dof_indices, dt)
    return {
        "test_id": test_id,
        "target_joint_rad": list(joints),
        "actual_joint_rad": actual,
        "settling_steps": steps,
        "timed_out": timed_out,
        "base_pose": xform_pose(stage, base_path),
        "tool0_pose": xform_pose(stage, tool_path),
    }


def version_text() -> str:
    path = ISAAC_ROOT / "VERSION"
    return path.read_text(encoding="utf-8").strip() if path.exists() else ""


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
    ensure_gravity(stage)
    roots = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError("expected one articulation root, found " + str(roots))
    base_path = unique_geometry_prim(stage, "base_link")
    tool_path = unique_geometry_prim(stage, "tool0")
    robot = Articulation(roots[0])
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()
    physics_dt = float(SimulationManager.get_physics_dt())
    mapping = map_canonical_dofs(list(robot.dof_names))
    dof_indices = [int(numbers(robot.get_dof_indices(str(row["isaac_dof_name"])))[0]) for row in mapping]
    rows = [measure(stage, robot, dof_indices, physics_dt, test_id, joints, base_path, tool_path) for test_id, joints in CONFIGURATIONS]
    timeline.stop()
    simulation_app.update()
    robot = Articulation(roots[0])
    timeline.play()
    simulation_app.update()
    rerun = measure(stage, robot, dof_indices, physics_dt, CONFIGURATIONS[0][0], CONFIGURATIONS[0][1], base_path, tool_path)
    gpu = ""
    try:
        gpu = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        gpu = "unavailable: " + type(exc).__name__
    payload = {
        "gate": "P-004",
        "source": "isaac",
        "source_commit": SOURCE_COMMIT,
        "isaac_version_file": version_text(),
        "execution_mode": "headless",
        "physics_device": "cpu",
        "os": platform.platform(),
        "gpu": gpu,
        "native_frame": "isaac_world",
        "quaternion_order": "wxyz",
        "handedness": "right",
        "up_axis": "Z",
        "normalization_applied": False,
        "tool0_observation_api": rows[0]["tool0_pose"]["pose_api"],
        "articulation_root": roots[0],
        "configurations": rows,
        "rerun": rerun,
    }
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "isaac_results.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
