"""시뮬레이터 타입을 쓰지 않는 Canonical Robot Model."""

from dataclasses import dataclass


@dataclass(frozen=True)
class JointLimits:
    """Canonical 단위의 한계. 위치와 속도는 radian이다."""

    lower: float
    upper: float
    velocity: float
    effort: float
    source_position_unit: str
    source_min_position: float
    source_max_position: float
    source_max_velocity: float


@dataclass(frozen=True)
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    limits: JointLimits | None


@dataclass(frozen=True)
class RobotModel:
    robot_id: str
    source_repository: str
    source_commit: str
    source_manifest: str
    model_identifier: str
    base_frame: str
    tcp_frame: str
    actuated_joint_order: tuple[str, ...]
    joints: tuple[Joint, ...]

    def joint_by_name(self, name: str) -> Joint:
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise KeyError(name)
