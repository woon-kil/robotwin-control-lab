"""Isaac 실행 없이 검사하는 UR10e canonical joint command 규칙.

Isaac DOF 순서를 canonical order로 가정하지 않는다.
"""

from __future__ import annotations

import math

CANONICAL_JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# P-001B baseline_configurations()와 같은 5개. core는 여기서 import하지 않는다.
CONFIGURATIONS = (
    ("A-zero", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("B-positive-1", (0.2, 0.3, 0.4, 0.5, 0.6, 0.7)),
    ("C-mixed-1", (0.5, -0.4, 0.3, -0.2, 0.6, -0.7)),
    ("E-fixed-1", (0.1, -0.2, 0.3, -0.4, 0.5, -0.6)),
    ("E-fixed-3", (0.01, 0.02, -0.03, 0.04, -0.05, 0.06)),
)

ENGINEERING_TARGET_DEGREES = 0.1
SOURCE_COMMIT = "7ce98d8c3512b9d0239da93b75de68ea64f61bd6"


class Ur10eCommandError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def validate_joint_targets(values: object) -> tuple[float, ...]:
    """개수와 유한성을 확인하고, drive를 쓰기 전에 거부한다."""
    if not isinstance(values, (list, tuple)) or len(values) != len(CANONICAL_JOINT_ORDER):
        length = "null" if values is None else str(len(values) if isinstance(values, (list, tuple)) else type(values).__name__)
        raise Ur10eCommandError("JOINT_COUNT", "expected 6 joints, got " + length)
    checked: list[float] = []
    for index, value in enumerate(values):
        number = float(value)
        if not math.isfinite(number):
            raise Ur10eCommandError("NON_FINITE", CANONICAL_JOINT_ORDER[index] + " target is not finite")
        checked.append(number)
    return tuple(checked)


def map_canonical_dofs(dof_names: list[str]) -> list[dict[str, object]]:
    """canonical name마다 Isaac DOF가 정확히 하나인지 확인한다."""
    if len(dof_names) != len(set(dof_names)):
        raise Ur10eCommandError("MAPPING", "Isaac DOF names are not unique")
    mapping: list[dict[str, object]] = []
    for canonical in CANONICAL_JOINT_ORDER:
        hits = [index for index, name in enumerate(dof_names) if name == canonical]
        if len(hits) != 1:
            raise Ur10eCommandError("MAPPING", "missing or duplicate canonical joint " + canonical)
        mapping.append(
            {
                "canonical_name": canonical,
                "isaac_dof_name": dof_names[hits[0]],
                "isaac_dof_index": hits[0],
            }
        )
    return mapping
