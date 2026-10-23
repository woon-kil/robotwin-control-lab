"""Headless Isaac Sim UR10e joint-control baseline.

Run on the machine that has Isaac Sim:

    ~/isaacsim/python.sh isaac/scripts/run_ur10e_joint_baseline.py --repo <repo>

Canonical commands stay in radians. The Isaac adapter does not compare Unity or Python FK.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--repo", required=True)
parser.add_argument("--isaac-root", default=str(Path.home() / "isaacsim"))
args = parser.parse_args()

REPO = Path(args.repo).resolve()
ISAAC_ROOT = Path(args.isaac_root).resolve()
sys.path.insert(0, str(REPO))

from isaac.ur10e_commands import (  # noqa: E402
    CANONICAL_JOINT_ORDER,
    CONFIGURATIONS,
    ENGINEERING_TARGET_DEGREES,
    SOURCE_COMMIT,
    Ur10eCommandError,
    map_canonical_dofs,
    validate_joint_targets,
)

sys.argv = [sys.argv[0]]

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.core.experimental.prims import Articulation, RigidPrim  # noqa: E402
from isaacsim.core.experimental.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

URDF_RELATIVE = Path("robot_description/generated") / SOURCE_COMMIT / "ur10e.urdf"
PACKAGE_RELATIVE = Path("robot_description/upstream") / SOURCE_COMMIT
EVIDENCE = REPO / "evidence" / "P-003"
USD_DIR = REPO / "isaac" / "generated" / "ur10e"
# URDFImporterConfig docstring: Nm/rad and Nm*s/rad. Not a UR10e controller specification.
DRIVE_STIFFNESS = 100000.0
DRIVE_DAMPING = 1000.0
TIMEOUT_SECONDS = 8.0
SETTLED_STEPS = 20
VELOCITY_EPSILON = 0.001
POSITION_DELTA_EPSILON = 1e-5
ROOT_DISPLACEMENT_LIMIT_M = 1e-3
TARGET_RADIANS = ENGINEERING_TARGET_DEGREES * math.pi / 180.0


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace").strip()


def urdf_joint_types(path: Path) -> dict[str, dict[str, str]]:
    root = ET.parse(path).getroot()
    found: dict[str, dict[str, str]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        if name not in CANONICAL_JOINT_ORDER:
            continue
        limit = joint.find("limit")
        found[name] = {
            "joint_type": joint.attrib.get("type", ""),
            "effort": "" if limit is None else limit.attrib.get("effort", ""),
        }
    return found


def numbers(value) -> list[float]:
    array = value.numpy() if hasattr(value, "numpy") else np.asarray(value)
    return [float(item) for item in np.reshape(array, -1)]


def find_named(stage, name: str) -> list[str]:
    return [str(prim.GetPath()) for prim in stage.Traverse() if prim.GetName() == name]


def find_articulation_roots(stage) -> list[str]:
    return [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]


def find_fixed_joints(stage) -> list[str]:
    return [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.FixedJoint)]


def usd_joint_attribute(stage, joint_name: str, attribute: str):
    matches = find_named(stage, joint_name)
    if len(matches) != 1:
        return None
    attr = stage.GetPrimAtPath(matches[0]).GetAttribute(attribute)
    if not attr.IsValid() or attr.Get() is None:
        return None
    value = attr.Get()
    if isinstance(value, str):
        return value
    return float(value)


def read_gravity(stage) -> dict[str, object]:
    scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if not scenes:
        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        return {
            "scene_path": "/PhysicsScene",
            "created_by_runner": True,
            "direction_xyz": [0.0, 0.0, -1.0],
            "magnitude": 9.81,
        }
    scene = UsdPhysics.Scene(scenes[0])
    direction = scene.GetGravityDirectionAttr().Get()
    magnitude = scene.GetGravityMagnitudeAttr().Get()
    return {
        "scene_path": str(scenes[0].GetPath()),
        "created_by_runner": False,
        "direction_xyz": None if direction is None else [float(direction[0]), float(direction[1]), float(direction[2])],
        "magnitude": None if magnitude is None else float(magnitude),
    }


def tool0_pose(path: str) -> dict[str, object]:
    prim = RigidPrim(path)
    position, orientation = prim.get_world_poses()
    xyz = numbers(position)
    quat = numbers(orientation)
    return {
        "frame": "isaac_world",
        "quaternion_order": "wxyz",
        "pose_api": "isaacsim.core.experimental.prims.RigidPrim.get_world_poses",
        "prim_path": path,
        "position_m": {"x": xyz[0], "y": xyz[1], "z": xyz[2]},
        "quaternion_wxyz": {"w": quat[0], "x": quat[1], "y": quat[2], "z": quat[3]},
    }


def tool0_pose_xform(stage, path: str) -> dict[str, object]:
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
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
        "position_m": {"x": float(translation[0]), "y": float(translation[1]), "z": float(translation[2])},
        "quaternion_wxyz": {
            "w": float(quat.GetReal()),
            "x": float(imaginary[0]),
            "y": float(imaginary[1]),
            "z": float(imaginary[2]),
        },
    }


def read_pose(stage, path: str) -> dict[str, object]:
    try:
        return tool0_pose(path)
    except Exception as exc:  # noqa: BLE001 - record the runtime API that actually worked
        pose = tool0_pose_xform(stage, path)
        pose["rigid_prim_error"] = type(exc).__name__
        return pose


def step_until_settled(robot, dof_indices: list[int], dt: float) -> tuple[list[float], list[float], int, bool]:
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


def command_targets(robot, dof_indices: list[int], targets: tuple[float, ...]) -> None:
    checked = validate_joint_targets(targets)
    robot.set_dof_position_targets(np.asarray(checked, dtype=np.float64).reshape(1, -1), dof_indices=dof_indices)
    robot.set_dof_velocity_targets(np.zeros((1, len(checked)), dtype=np.float64), dof_indices=dof_indices)


def regulate_from_command(robot, dof_indices: list[int], targets: tuple[float, ...]) -> None:
    """Place each configuration on its command, then hold that command.

    Articulation.reset_to_default_state() does nothing until set_default_state().
    Without an explicit state write, the next configuration inherits the previous
    PhysX joint state. Feedback is still read from the physics tensor after settling.
    """
    checked = validate_joint_targets(targets)
    values = np.asarray(checked, dtype=np.float64).reshape(1, -1)
    zeros = np.zeros_like(values)
    robot.set_dof_velocities(zeros, dof_indices=dof_indices)
    robot.set_dof_positions(values, dof_indices=dof_indices)
    command_targets(robot, dof_indices, targets)


def read_solver_iterations(robot, stage) -> tuple[int | None, int | None, str]:
    position = None
    velocity = None
    for prim in stage.Traverse():
        position_attr = prim.GetAttribute("physxArticulation:solverPositionIterationCount")
        velocity_attr = prim.GetAttribute("physxArticulation:solverVelocityIterationCount")
        if position_attr.IsValid() and position_attr.Get() is not None:
            position = int(position_attr.Get())
        if velocity_attr.IsValid() and velocity_attr.Get() is not None:
            velocity = int(velocity_attr.Get())
    if position is None and velocity is None:
        return None, None, "physxArticulation:solverPositionIterationCount is not authored"
    try:
        position_iterations, velocity_iterations = robot.get_solver_iteration_counts()
        position = int(np.reshape(position_iterations.numpy(), -1)[0])
        velocity = int(np.reshape(velocity_iterations.numpy(), -1)[0])
        return position, velocity, "Articulation.get_solver_iteration_counts"
    except Exception:
        return position, velocity, "usd attribute"


def reject_before_write(robot, dof_indices: list[int], values, expected_code: str) -> bool:
    before = numbers(robot.get_dof_positions(dof_indices=dof_indices))
    rejected = False
    try:
        command_targets(robot, dof_indices, values)
    except Ur10eCommandError as exc:
        rejected = exc.code == expected_code
    after = numbers(robot.get_dof_positions(dof_indices=dof_indices))
    return rejected and max(abs(after[index] - before[index]) for index in range(len(before))) == 0.0


def environment_probe() -> dict[str, str]:
    os_release = {}
    release_path = Path("/etc/os-release")
    if release_path.exists():
        for line in release_path.read_text(encoding="utf-8").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                os_release[key] = value.strip().strip('"')
    gpu = ""
    try:
        gpu = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        gpu = "unavailable: " + type(exc).__name__
    version_path = ISAAC_ROOT / "VERSION"
    return {
        "os": os_release.get("PRETTY_NAME", platform.platform()),
        "kernel": platform.platform(),
        "gpu": gpu,
        "python": sys.version.replace("\n", " "),
        "isaac_version_file": read_text(version_path) if version_path.exists() else "",
    }


def write_evidence(payload: dict[str, object], summary: str, environment: str, test_log: str) -> None:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "joint_results.json").write_text(json.dumps(payload["joint_results"], indent=2) + "\n", encoding="utf-8")
    (EVIDENCE / "import_report.json").write_text(json.dumps(payload["import_report"], indent=2) + "\n", encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(summary, encoding="utf-8")
    (EVIDENCE / "environment.md").write_text(environment, encoding="utf-8")
    (EVIDENCE / "test_results.txt").write_text(test_log, encoding="utf-8")


def main() -> int:
    urdf_path = REPO / URDF_RELATIVE
    package_root = REPO / PACKAGE_RELATIVE
    probe = environment_probe()
    failures: list[str] = []
    joint_rows: list[dict[str, object]] = []
    import_report: dict[str, object] = {
        "source_repository": "UniversalRobots/Universal_Robots_ROS2_Description",
        "source_commit": SOURCE_COMMIT,
        "source_urdf": str(URDF_RELATIVE),
        "source_urdf_sha256": sha256(urdf_path) if urdf_path.is_file() else "",
        "robot": "ur10e",
        "isaac_version_file": probe["isaac_version_file"],
        "execution_mode": "headless",
    }
    try:
        if not urdf_path.is_file():
            raise FileNotFoundError(str(urdf_path))
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
        gravity = read_gravity(stage)
        roots = find_articulation_roots(stage)
        tool_paths = find_named(stage, "tool0")
        if len(roots) != 1:
            raise RuntimeError("expected one articulation root, found " + str(roots))
        if len(tool_paths) != 1:
            raise RuntimeError("expected one tool0, found " + str(tool_paths))
        robot = Articulation(roots[0])
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        simulation_app.update()
        dof_names = list(robot.dof_names)
        mapping = map_canonical_dofs(dof_names)
        api_indices = []
        for row in mapping:
            api_index = int(numbers(robot.get_dof_indices(str(row["isaac_dof_name"])))[0])
            if api_index != row["isaac_dof_index"]:
                raise Ur10eCommandError("MAPPING", "DOF list index does not match get_dof_indices")
            api_indices.append(api_index)
        urdf_types = urdf_joint_types(urdf_path)
        try:
            raw_drive_types = robot.get_dof_drive_types(dof_indices=api_indices)
            drive_type_values = [str(item) for item in np.asarray(raw_drive_types, dtype=object).reshape(-1)]
        except Exception as exc:  # noqa: BLE001
            drive_type_values = ["unreadable:" + type(exc).__name__]
        for row, drive_type in zip(mapping, drive_type_values, strict=False):
            source = urdf_types.get(str(row["canonical_name"]), {})
            row["joint_type"] = source.get("joint_type", "")
            row["urdf_effort"] = source.get("effort", "")
            row["isaac_drive_type"] = drive_type
            row["usd_drive_type"] = usd_joint_attribute(stage, str(row["canonical_name"]), "drive:angular:physics:type")
            row["usd_drive_stiffness"] = usd_joint_attribute(stage, str(row["canonical_name"]), "drive:angular:physics:stiffness")
            row["usd_drive_damping"] = usd_joint_attribute(stage, str(row["canonical_name"]), "drive:angular:physics:damping")
            row["usd_drive_max_force"] = usd_joint_attribute(stage, str(row["canonical_name"]), "drive:angular:physics:maxForce")
        if len(drive_type_values) != len(mapping):
            for row in mapping:
                source = urdf_types.get(str(row["canonical_name"]), {})
                row["joint_type"] = source.get("joint_type", "")
                row["urdf_effort"] = source.get("effort", "")
                row["isaac_drive_type"] = drive_type_values[0] if drive_type_values else ""
        invalid_count = reject_before_write(robot, api_indices, [0.1, 0.1, 0.1, 0.1, 0.1], "JOINT_COUNT")
        invalid_finite = reject_before_write(robot, api_indices, [0.0, 0.0, math.nan, 0.0, 0.0, 0.0], "NON_FINITE")
        unknown_mapping = False
        try:
            map_canonical_dofs(["shoulder_pan_joint"])
        except Ur10eCommandError as exc:
            unknown_mapping = exc.code == "MAPPING"
        duplicate_mapping = False
        try:
            map_canonical_dofs(list(CANONICAL_JOINT_ORDER) + ["shoulder_pan_joint"])
        except Ur10eCommandError as exc:
            duplicate_mapping = exc.code == "MAPPING"
        dt = float(SimulationManager.get_physics_dt())
        solver_position, solver_velocity, solver_source = read_solver_iterations(robot, stage)
        try:
            stiffness, damping = robot.get_dof_gains(dof_indices=api_indices)
            max_effort = robot.get_dof_max_efforts(dof_indices=api_indices)
            stiffness_values = numbers(stiffness)
            damping_values = numbers(damping)
            effort_values = numbers(max_effort)
        except Exception as exc:  # noqa: BLE001
            stiffness_values = []
            damping_values = []
            effort_values = []
            failures.append("drive readback unreadable: " + type(exc).__name__)
        root_start = numbers(robot.get_world_poses()[0])
        measured_rows = []
        for test_id, joints in CONFIGURATIONS:
            regulate_from_command(robot, api_indices, joints)
            simulation_app.update()
            start = numbers(robot.get_world_poses()[0])
            actual, velocity, steps, timed_out = step_until_settled(robot, api_indices, dt)
            end = numbers(robot.get_world_poses()[0])
            displacement = math.dist(start, end)
            error = [actual[index] - joints[index] for index in range(len(joints))]
            max_abs = max(abs(item) for item in error)
            pose = read_pose(stage, tool_paths[0])
            finite_pose = all(math.isfinite(value) for value in pose["position_m"].values()) and all(
                math.isfinite(value) for value in pose["quaternion_wxyz"].values()
            )
            passed = (
                not timed_out
                and finite_pose
                and max_abs < TARGET_RADIANS
                and displacement <= ROOT_DISPLACEMENT_LIMIT_M
                and actual != list(joints)
            )
            if not passed:
                failures.append(test_id + " max_abs_error_rad=" + format(max_abs, ".17g"))
            measured_rows.append(
                {
                    "test_id": test_id,
                    "target_joint_rad": list(joints),
                    "actual_joint_rad": actual,
                    "error_joint_rad": error,
                    "max_abs_error_rad": max_abs,
                    "max_abs_error_deg": max_abs * 180.0 / math.pi,
                    "settling_steps": steps,
                    "settling_time": steps * dt,
                    "timeout": TIMEOUT_SECONDS,
                    "timed_out": timed_out,
                    "root_displacement_m": displacement,
                    "final_velocity_rad_s": velocity,
                    "tool0_pose": pose,
                    "result": "PASS" if passed else "FAIL",
                }
            )
        regulate_from_command(robot, api_indices, CONFIGURATIONS[0][1])
        simulation_app.update()
        rerun_actual, _, _, _ = step_until_settled(robot, api_indices, dt)
        first_actual = measured_rows[0]["actual_joint_rad"]
        rerun_error = max(abs(rerun_actual[index] - first_actual[index]) for index in range(len(first_actual)))
        global_max = max(row["max_abs_error_rad"] for row in measured_rows)
        timeouts = sum(1 for row in measured_rows if row["timed_out"])
        root_motion = max(row["root_displacement_m"] for row in measured_rows)
        if not invalid_count:
            failures.append("invalid joint count was not rejected")
        if not invalid_finite:
            failures.append("non-finite input was not rejected")
        if not unknown_mapping or not duplicate_mapping:
            failures.append("unknown or duplicate mapping was not rejected")
        if timeouts:
            failures.append("timeout count " + str(timeouts))
        if global_max >= TARGET_RADIANS:
            failures.append("global max exceeds 0.1 deg")
        if root_motion > ROOT_DISPLACEMENT_LIMIT_M:
            failures.append("root moved " + format(root_motion, ".17g") + " m")
        result = "FAIL" if failures else "PASS"
        up_axis = UsdGeom.GetStageUpAxis(stage)
        import_report.update(
            {
                "generated_asset": generated,
                "articulation_root": roots[0],
                "fixed_base": True,
                "fixed_joints": find_fixed_joints(stage),
                "tool0_path": tool_paths[0],
                "canonical_joint_order": list(CANONICAL_JOINT_ORDER),
                "joint_mapping": mapping,
                "dof_names": dof_names,
                "importer": {
                    "class": "isaacsim.asset.importer.urdf.URDFImporter",
                    "fix_base": True,
                    "merge_fixed_joints": False,
                    "allow_self_collision": False,
                    "joint_drive_type": "force",
                    "joint_target_type": "position",
                    "override_joint_stiffness_nm_per_rad": DRIVE_STIFFNESS,
                    "override_joint_damping_nm_s_per_rad": DRIVE_DAMPING,
                    "run_asset_transformer": False,
                    "run_multi_physics_conversion": True,
                    "ros_package": "ur_description",
                },
            }
        )
        joint_results = {
            "gate": "P-003",
            "result": result,
            "source_commit": SOURCE_COMMIT,
            "canonical_joint_order": list(CANONICAL_JOINT_ORDER),
            "engineering_target_deg": ENGINEERING_TARGET_DEGREES,
            "global_max_joint_error_rad": global_max,
            "global_max_joint_error_deg": global_max * 180.0 / math.pi,
            "timeouts": timeouts,
            "deterministic_rerun_max_abs_rad": rerun_error,
            "invalid_joint_count_rejected": invalid_count,
            "non_finite_rejected": invalid_finite,
            "unknown_mapping_rejected": unknown_mapping and duplicate_mapping,
            "root_displacement_max_m": root_motion,
            "initial_condition": "teleport dof positions to the commanded configuration and zero dof velocities, then set position targets",
            "physics_dt_s": dt,
            "solver_position_iterations": solver_position,
            "solver_velocity_iterations": solver_velocity,
            "solver_iteration_source": solver_source,
            "drive_stiffness_readback": stiffness_values,
            "drive_damping_readback": damping_values,
            "drive_max_effort_readback": effort_values,
            "configurations": measured_rows,
        }
        summary = render_summary(joint_results, probe, roots[0])
        environment = render_environment(probe, gravity, up_axis, dt, solver_position, solver_velocity, solver_source, stiffness_values, damping_values, effort_values)
        test_log = render_log(joint_results)
        write_evidence({"joint_results": joint_results, "import_report": import_report}, summary, environment, test_log)
        return 0 if result == "PASS" else 1
    except Exception as exc:  # noqa: BLE001
        failures.append(type(exc).__name__ + ": " + str(exc))
        joint_results = {
            "gate": "P-003",
            "result": "FAIL",
            "source_commit": SOURCE_COMMIT,
            "engineering_target_deg": ENGINEERING_TARGET_DEGREES,
            "failures": failures,
            "configurations": joint_rows,
        }
        import_report["error"] = failures[-1]
        write_evidence(
            {"joint_results": joint_results, "import_report": import_report},
            "# P-003 Isaac UR10e joint control\n\n```text\nResult\nFAIL\n\n" + failures[-1] + "\n```\n",
            render_environment(probe, {}, "", None, None, None, "", [], [], []),
            "result FAIL\n" + failures[-1] + "\n",
        )
        return 1
    finally:
        simulation_app.close()


def render_summary(results: dict[str, object], probe: dict[str, str], root: str) -> str:
    return "\n".join(
        [
            "# P-003 Isaac UR10e joint control",
            "",
            "```text",
            "Gate",
            "P-003",
            "",
            "Result",
            str(results["result"]),
            "",
            "Robot",
            "UR10e Golden Reference",
            "",
            "Source commit",
            SOURCE_COMMIT,
            "",
            "Isaac Sim",
            probe["isaac_version_file"],
            "",
            "Articulation root",
            root,
            "",
            "Configurations",
            str(len(results["configurations"])),
            "",
            "Global max joint error",
            format(float(results["global_max_joint_error_rad"]), ".17g") + " rad",
            format(float(results["global_max_joint_error_deg"]), ".17g") + " deg",
            "",
            "Engineering target",
            "< 0.1 deg",
            "",
            "Timeouts",
            str(results["timeouts"]),
            "",
            "Known limitations",
            "official nominal UR10e model",
            "not robot-specific calibration",
            "not compared with actual UR10e",
            "not yet compared with Unity",
            "not yet compared with Python FK",
            "no physical robot accuracy claim",
            "Isaac controller/drive is simulation test configuration",
            "```",
            "",
            "이 숫자는 Isaac PhysX articulation feedback이다. Python FK 정확도나 Unity와의 우열이 아니다.",
            "",
        ]
    )


def render_environment(probe, gravity, up_axis, dt, solver_position, solver_velocity, solver_source, stiffness, damping, effort) -> str:
    return "\n".join(
        [
            "# P-003 environment",
            "",
            "```text",
            "OS",
            probe.get("os", ""),
            "",
            "Kernel",
            probe.get("kernel", ""),
            "",
            "GPU",
            probe.get("gpu", ""),
            "",
            "Python",
            probe.get("python", ""),
            "",
            "Isaac Sim version",
            probe.get("isaac_version_file", ""),
            "",
            "Isaac root",
            str(ISAAC_ROOT),
            "",
            "Execution mode",
            "headless",
            "",
            "Physics device",
            "cpu",
            "",
            "Stage up axis",
            str(up_axis),
            "",
            "Physics timestep",
            "" if dt is None else format(float(dt), ".17g"),
            "",
            "Gravity",
            json.dumps(gravity),
            "",
            "Solver position iterations",
            "" if solver_position is None else format(float(solver_position), ".17g"),
            "",
            "Solver velocity iterations",
            "" if solver_velocity is None else format(float(solver_velocity), ".17g"),
            "",
            "Solver iteration source",
            "" if solver_source is None else str(solver_source),
            "",
            "Source URDF",
            str(URDF_RELATIVE),
            "",
            "Source commit",
            SOURCE_COMMIT,
            "",
            "PROJECT TEST PARAMETER",
            "drive stiffness Nm/rad",
            format(DRIVE_STIFFNESS, ".17g"),
            "drive damping Nm*s/rad",
            format(DRIVE_DAMPING, ".17g"),
            "stiffness readback",
            json.dumps(stiffness),
            "damping readback",
            json.dumps(damping),
            "max effort readback",
            json.dumps(effort),
            "settled steps",
            str(SETTLED_STEPS),
            "velocity epsilon rad/s",
            format(VELOCITY_EPSILON, ".17g"),
            "position delta epsilon rad",
            format(POSITION_DELTA_EPSILON, ".17g"),
            "timeout seconds",
            format(TIMEOUT_SECONDS, ".17g"),
            "",
            "Initial condition",
            "teleport dof positions to the commanded configuration and zero dof velocities, then set position targets",
            "```",
            "",
            "Drive stiffness와 damping은 URDFImporterConfig에 Nm/rad, Nm*s/rad로 적힌 PROJECT TEST PARAMETER다. UR10e controller capability가 아니다.",
            "USD drive:angular:physics:stiffness와 damping은 import_report.json의 joint_mapping에 있다. drive_stiffness_readback은 Articulation.get_dof_gains 값이다.",
            "solver iteration attribute가 stage에 없으면 스키마 기본값을 사용한 것처럼 적지 않는다.",
            "Quaternion은 RigidPrim.get_world_poses 문서의 wxyz를 사용한다. tensor 원본 xyzw를 그대로 적지 않는다.",
            "tool0 frame은 isaac_world다. canonical base_link로 변환하지 않았다.",
            "",
        ]
    )


def render_log(results: dict[str, object]) -> str:
    lines = [
        "result " + str(results["result"]),
        "global_max_joint_error_rad " + format(float(results["global_max_joint_error_rad"]), ".17g"),
        "global_max_joint_error_deg " + format(float(results["global_max_joint_error_deg"]), ".17g"),
        "timeouts " + str(results["timeouts"]),
        "deterministic_rerun_max_abs_rad " + format(float(results["deterministic_rerun_max_abs_rad"]), ".17g"),
    ]
    for row in results["configurations"]:
        lines.append(
            row["test_id"]
            + " "
            + row["result"]
            + " steps="
            + str(row["settling_steps"])
            + " max_abs_error_rad="
            + format(float(row["max_abs_error_rad"]), ".17g")
            + " timed_out="
            + str(row["timed_out"]).lower()
        )
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    raise SystemExit(main())
