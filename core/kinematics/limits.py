"""Joint limit은 FK와 분리된 검사다. 관절값을 바꾸지 않는다."""

from core.model.types import RobotModel


def limit_violations(
    robot_model: RobotModel,
    joint_positions: tuple[float, ...],
) -> tuple[str, ...]:
    """한계 밖 관절 이름을 canonical order로 반환한다. 입력은 수정하지 않는다."""
    if len(joint_positions) != len(robot_model.actuated_joint_order):
        raise ValueError("joint_positions length does not match actuated joint order")
    violated: list[str] = []
    for name, position in zip(robot_model.actuated_joint_order, joint_positions, strict=True):
        limits = robot_model.joint_by_name(name).limits
        if limits is None:
            continue
        if position < limits.lower or position > limits.upper:
            violated.append(name)
    return tuple(violated)
