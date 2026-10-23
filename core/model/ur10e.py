"""핀된 UR10e nominal description에서 Canonical Model을 읽는다."""

import math
from pathlib import Path

from core.errors import KinematicsError
from core.model.types import Joint, JointLimits, RobotModel

SOURCE_REPOSITORY = "UniversalRobots/Universal_Robots_ROS2_Description"
SOURCE_COMMIT = "7ce98d8c3512b9d0239da93b75de68ea64f61bd6"
SOURCE_MANIFEST = "robot_description/SOURCE_MANIFEST.md"
ROBOT_ID = "ur10e"
BASE_FRAME = "base_link"
TCP_FRAME = "tool0"

# xacro 직렬 체인 순서. dict 순서를 쓰지 않는다.
ACTUATED_JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

_KINEMATICS_KEY = {
    "shoulder_pan_joint": "shoulder",
    "shoulder_lift_joint": "upper_arm",
    "elbow_joint": "forearm",
    "wrist_1_joint": "wrist_1",
    "wrist_2_joint": "wrist_2",
    "wrist_3_joint": "wrist_3",
}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def source_root() -> Path:
    return repo_root() / "robot_description" / "upstream" / SOURCE_COMMIT


def degrees_to_radians(degrees: float) -> float:
    """YAML `!degrees`를 canonical radian으로 바꾼다. 암묵적 혼용을 두지 않는다."""
    return degrees * math.pi / 180.0


def load_ur10e_model(root: Path | None = None) -> RobotModel:
    source = source_root() if root is None else root
    kinematics = _load_nested(source / "config" / "ur10e" / "default_kinematics.yaml")
    limits_file = _load_nested(source / "config" / "ur10e" / "joint_limits.yaml")
    kin = kinematics["kinematics"]
    limit_map = limits_file["joint_limits"]
    limits_by_name = {name: _limits_from_source(limit_map[name]) for name in ACTUATED_JOINT_ORDER}
    wrist_3_type = _wrist_3_type(limit_map["wrist_3_joint"])
    pi = math.pi
    joints = (
        Joint(
            "base_joint",
            "fixed",
            "world",
            "base_link",
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            None,
        ),
        Joint(
            "base_link-base_fixed_joint",
            "fixed",
            "base_link",
            "base",
            (0.0, 0.0, 0.0),
            (0.0, 0.0, pi),
            (0.0, 0.0, 0.0),
            None,
        ),
        Joint(
            "base_link-base_link_inertia",
            "fixed",
            "base_link",
            "base_link_inertia",
            (0.0, 0.0, 0.0),
            (0.0, 0.0, pi),
            (0.0, 0.0, 0.0),
            None,
        ),
        *_revolute_joints(kin, limits_by_name, wrist_3_type),
        Joint(
            "wrist_3_link-ft_frame",
            "fixed",
            "wrist_3_link",
            "ft_frame",
            (0.0, 0.0, 0.0),
            (pi, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            None,
        ),
        Joint(
            "wrist_3-flange",
            "fixed",
            "wrist_3_link",
            "flange",
            (0.0, 0.0, 0.0),
            (0.0, -pi / 2.0, -pi / 2.0),
            (0.0, 0.0, 0.0),
            None,
        ),
        Joint(
            "flange-tool0",
            "fixed",
            "flange",
            "tool0",
            (0.0, 0.0, 0.0),
            (pi / 2.0, 0.0, pi / 2.0),
            (0.0, 0.0, 0.0),
            None,
        ),
    )
    return RobotModel(
        robot_id=ROBOT_ID,
        source_repository=SOURCE_REPOSITORY,
        source_commit=SOURCE_COMMIT,
        source_manifest=SOURCE_MANIFEST,
        model_identifier=f"ur10e-nominal-{SOURCE_COMMIT}",
        base_frame=BASE_FRAME,
        tcp_frame=TCP_FRAME,
        actuated_joint_order=ACTUATED_JOINT_ORDER,
        joints=joints,
    )


def _revolute_joints(
    kinematics: dict,
    limits_by_name: dict[str, JointLimits],
    wrist_3_type: str,
) -> tuple[Joint, ...]:
    parents = {
        "shoulder_pan_joint": "base_link_inertia",
        "shoulder_lift_joint": "shoulder_link",
        "elbow_joint": "upper_arm_link",
        "wrist_1_joint": "forearm_link",
        "wrist_2_joint": "wrist_1_link",
        "wrist_3_joint": "wrist_2_link",
    }
    children = {
        "shoulder_pan_joint": "shoulder_link",
        "shoulder_lift_joint": "upper_arm_link",
        "elbow_joint": "forearm_link",
        "wrist_1_joint": "wrist_1_link",
        "wrist_2_joint": "wrist_2_link",
        "wrist_3_joint": "wrist_3_link",
    }
    built: list[Joint] = []
    for name in ACTUATED_JOINT_ORDER:
        segment = kinematics[_KINEMATICS_KEY[name]]
        joint_type = wrist_3_type if name == "wrist_3_joint" else "revolute"
        limits = limits_by_name[name] if joint_type == "revolute" else None
        built.append(
            Joint(
                name=name,
                joint_type=joint_type,
                parent=parents[name],
                child=children[name],
                origin_xyz=(float(segment["x"]), float(segment["y"]), float(segment["z"])),
                origin_rpy=(float(segment["roll"]), float(segment["pitch"]), float(segment["yaw"])),
                axis=(0.0, 0.0, 1.0),
                limits=limits,
            )
        )
    return tuple(built)


def _wrist_3_type(raw: dict) -> str:
    """ur_common.xacro: has_position_limits이면 revolute, 아니면 continuous."""
    if bool(raw["has_position_limits"]) is True:
        return "revolute"
    return "continuous"


def _limits_from_source(raw: dict) -> JointLimits:
    min_position = raw["min_position"]
    max_position = raw["max_position"]
    max_velocity = raw["max_velocity"]
    for label, value in (
        ("min_position", min_position),
        ("max_position", max_position),
        ("max_velocity", max_velocity),
    ):
        if not isinstance(value, tuple) or value[0] != "degrees":
            raise KinematicsError("INVALID_MODEL", f"{label} is not tagged !degrees")
    min_deg = float(min_position[1])
    max_deg = float(max_position[1])
    vel_deg = float(max_velocity[1])
    return JointLimits(
        lower=degrees_to_radians(min_deg),
        upper=degrees_to_radians(max_deg),
        velocity=degrees_to_radians(vel_deg),
        effort=float(raw["max_effort"]),
        source_position_unit="degrees",
        source_min_position=min_deg,
        source_max_position=max_deg,
        source_max_velocity=vel_deg,
    )


def _load_nested(path: Path) -> dict:
    if not path.is_file():
        raise KinematicsError("INVALID_MODEL", f"missing source file {path}")
    root: dict = {}
    stack: list[tuple[int, dict]] = [(-1, root)]
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        commentless = raw_line.split("#", 1)[0].rstrip()
        if not commentless.strip():
            continue
        indent = len(commentless) - len(commentless.lstrip(" "))
        key, separator, value = commentless.strip().partition(":")
        if separator != ":":
            raise KinematicsError("INVALID_MODEL", f"cannot parse {path.name}: {raw_line}")
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if value.strip() == "":
            node: dict = {}
            parent[key.strip()] = node
            stack.append((indent, node))
        else:
            parent[key.strip()] = _scalar(value.strip())
    return root


def _scalar(value: str):
    if value.startswith("!degrees"):
        return ("degrees", float(value.split(None, 1)[1]))
    if value == "true":
        return True
    if value == "false":
        return False
    try:
        return float(value)
    except ValueError:
        return value
