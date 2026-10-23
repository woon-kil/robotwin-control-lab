"""A-zero에서 gravity, solver iteration, stiffness가 settled joint에 주는 영향을 측정한다.

최종 acceptance run이 아니다. drive damping과 timestep은 바꾸지 않는다.
evidence/P-004와 fix-attempt1은 쓰지 않는다. RigidPrim은 만들지 않는다.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--repo", required=True)
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
from isaacsim.asset.importer.utils.impl.asset_utils import fix_articulation_root_for_fixed_base  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.experimental.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, UsdPhysics  # noqa: E402

URDF_RELATIVE = Path("robot_description/generated") / SOURCE_COMMIT / "ur10e.urdf"
PACKAGE_RELATIVE = Path("robot_description/upstream") / SOURCE_COMMIT
EVIDENCE = REPO / "evidence" / "P-004" / "fix-attempt2"
USD_DIR = REPO / "isaac" / "generated" / "ur10e"
DRIVE_STIFFNESS = 100000.0
DRIVE_DAMPING = 1000.0
TIMEOUT_SECONDS = 8.0
SETTLED_STEPS = 20
VELOCITY_EPSILON = 0.001
POSITION_DELTA_EPSILON = 1e-5
TARGET = CONFIGURATIONS[0][1]


def numbers(value) -> list[float]:
    array = value.numpy() if hasattr(value, "numpy") else np.asarray(value)
    return [float(item) for item in np.reshape(array, -1)]


def fresh(timeline, root: str):
    timeline.stop()
    simulation_app.update()
    robot = Articulation(root)
    timeline.play()
    simulation_app.update()
    return robot


def regulate(robot, dof_indices: list[int], targets: tuple[float, ...]) -> None:
    checked = validate_joint_targets(targets)
    values = np.asarray(checked, dtype=np.float64).reshape(1, -1)
    zeros = np.zeros_like(values)
    robot.set_dof_velocities(zeros, dof_indices=dof_indices)
    robot.set_dof_positions(values, dof_indices=dof_indices)
    robot.set_dof_position_targets(values, dof_indices=dof_indices)
    robot.set_dof_velocity_targets(zeros, dof_indices=dof_indices)


def settle(robot, dof_indices: list[int], dt: float) -> tuple[list[float], list[float], int, bool]:
    previous = numbers(robot.get_dof_positions(dof_indices=dof_indices))
    stable = 0
    steps = 0
    elapsed = 0.0
    actual = previous
    velocity = previous
    while elapsed < TIMEOUT_SECONDS:
        simulation_app.update()
        steps += 1
        elapsed += dt
        actual = numbers(robot.get_dof_positions(dof_indices=dof_indices))
        velocity = numbers(robot.get_dof_velocities(dof_indices=dof_indices))
        if any(not np.isfinite(item) for item in actual + velocity):
            return actual, velocity, steps, True
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


def authored_solver(stage, path: str) -> dict[str, object]:
    prim = stage.GetPrimAtPath(path)
    row: dict[str, object] = {"prim_path": path, "exists": prim.IsValid()}
    for name, key in (
        ("physxArticulation:solverPositionIterationCount", "position"),
        ("physxArticulation:solverVelocityIterationCount", "velocity"),
    ):
        attr = prim.GetAttribute(name) if prim.IsValid() else None
        valid = bool(attr is not None and attr.IsValid())
        authored = bool(valid and attr.HasAuthoredValueOpinion())
        value = attr.Get() if valid else None
        row[key] = {"authored": authored, "value": None if value is None else int(value)}
    return row


def read_efforts(robot, dof_indices: list[int]) -> dict[str, object]:
    payload: dict[str, object] = {}
    readers = {
        "dof_efforts": robot.get_dof_efforts,
        "max_efforts": robot.get_dof_max_efforts,
        "gravity_compensation": robot.get_dof_gravity_compensation_forces,
    }
    for key, reader in readers.items():
        try:
            payload[key] = numbers(reader(dof_indices=dof_indices))
        except Exception as exc:
            payload[key] = {"error": type(exc).__name__ + ": " + str(exc)}
    efforts = payload.get("dof_efforts")
    limits = payload.get("max_efforts")
    if isinstance(efforts, list) and isinstance(limits, list) and len(efforts) == len(limits):
        payload["abs_effort_over_limit"] = [
            None if limit == 0 else abs(effort) / abs(limit) for effort, limit in zip(efforts, limits)
        ]
    return payload


def measure(robot, dof_indices: list[int], dt: float) -> dict[str, object]:
    regulate(robot, dof_indices, TARGET)
    actual, velocity, steps, timed_out = settle(robot, dof_indices, dt)
    gains_stiffness, gains_damping = robot.get_dof_gains(dof_indices=dof_indices)
    try:
        position_counts, velocity_counts = robot.get_solver_iteration_counts()
        solver = {"position": numbers(position_counts), "velocity": numbers(velocity_counts), "error": None}
    except Exception as exc:
        solver = {"position": None, "velocity": None, "error": type(exc).__name__ + ": " + str(exc)}
    return {
        "actual_joint_rad": actual,
        "target_error_rad": [actual[index] - TARGET[index] for index in range(len(actual))],
        "velocity_rad_s": velocity,
        "max_abs_velocity_rad_s": max(abs(item) for item in velocity),
        "settling_steps": steps,
        "timed_out": timed_out,
        "non_finite": any(not np.isfinite(item) for item in actual + velocity),
        "stiffness_readback": numbers(gains_stiffness),
        "damping_readback": numbers(gains_damping),
        "solver_iteration_readback": solver,
        "efforts": read_efforts(robot, dof_indices),
    }


def scene_prim(stage):
    scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if not scenes:
        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        return scene.GetPrim()
    return scenes[0]


def main() -> int:
    SimulationManager.set_physics_sim_device("cpu")
    USD_DIR.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(REPO / URDF_RELATIVE),
        usd_path=str(USD_DIR),
        merge_fixed_joints=False,
        merge_mesh=False,
        collision_from_visuals=False,
        allow_self_collision=False,
        ros_package_paths=[{"name": "ur_description", "path": str(REPO / PACKAGE_RELATIVE)}],
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
    scene = scene_prim(stage)
    gravity = UsdPhysics.Scene(scene)
    relocated = int(fix_articulation_root_for_fixed_base(stage))
    roots = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError("expected one articulation root, found " + str(roots))
    root = roots[0]
    timeline = omni.timeline.get_timeline_interface()
    robot = fresh(timeline, root)
    indices = [int(numbers(robot.get_dof_indices(str(row["isaac_dof_name"])))[0]) for row in map_canonical_dofs(list(robot.dof_names))]
    dt = float(SimulationManager.get_physics_dt())
    solver_before = authored_solver(stage, root)
    rows = []

    def record(name: str, family: str) -> None:
        row = measure(robot, indices, dt)
        row["name"] = name
        row["family"] = family
        row["gravity_magnitude"] = gravity.GetGravityMagnitudeAttr().Get()
        rows.append(row)
        error = row["target_error_rad"]
        print(name, "SL", error[1], "EL", error[2], "timeout", row["timed_out"], "steps", row["settling_steps"])

    record("baseline", "baseline")
    gravity.GetGravityMagnitudeAttr().Set(0.0)
    robot = fresh(timeline, root)
    record("gravity_off", "gravity")
    gravity.GetGravityMagnitudeAttr().Set(9.81)
    robot = fresh(timeline, root)
    for stiffness in (200000.0, 400000.0):
        robot.set_dof_gains(stiffness, DRIVE_DAMPING, dof_indices=indices)
        record("stiffness_" + format(stiffness, ".0f"), "stiffness")
        robot = fresh(timeline, root)
    robot.set_dof_gains(DRIVE_STIFFNESS, DRIVE_DAMPING, dof_indices=indices)
    solver_prims = []
    for prim in stage.Traverse():
        attr = prim.GetAttribute("physxArticulation:solverPositionIterationCount")
        if attr.IsValid() and attr.Get() is not None:
            solver_prims.append({"path": str(prim.GetPath()), "position": int(attr.Get())})
    for count in (8, 16, 32):
        try:
            robot.set_solver_iteration_counts([count])
        except Exception as exc:
            rows.append({"name": "solver_position_" + str(count), "family": "solver_position_iterations", "error": type(exc).__name__ + ": " + str(exc), "solver_prims_with_value": solver_prims})
            print("solver", count, type(exc).__name__)
            break
        robot = fresh(timeline, root)
        robot.set_dof_gains(DRIVE_STIFFNESS, DRIVE_DAMPING, dof_indices=indices)
        record("solver_position_" + str(count), "solver_position_iterations")
    payload = {
        "gate": "P-004",
        "diagnostic": "fix-attempt2",
        "acceptance_run": False,
        "configuration": "A-zero",
        "source_commit": SOURCE_COMMIT,
        "articulation_root": root,
        "articulation_root_relocated_count": relocated,
        "physics_dt_s": dt,
        "baseline_stiffness": DRIVE_STIFFNESS,
        "baseline_damping": DRIVE_DAMPING,
        "solver_attribute_before_sweep": solver_before,
        "rows": rows,
    }
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "equilibrium_analysis.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    simulation_app.close()
    print("WROTE", EVIDENCE / "equilibrium_analysis.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
