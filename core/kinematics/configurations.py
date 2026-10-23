"""P-001B의 고정 테스트 관절 벡터. 난수 seed는 쓰지 않는다."""

import math

_NEAR = 0.99


def baseline_configurations() -> tuple[dict[str, object], ...]:
    """각 벡터는 canonical actuated order의 radian이다."""
    near_full = _NEAR * 2.0 * math.pi
    near_elbow = _NEAR * math.pi
    rows = (
        ("A-zero", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ("B-positive-1", (0.2, 0.3, 0.4, 0.5, 0.6, 0.7)),
        ("B-positive-2", (1.0, 0.5, 0.8, 0.2, 0.4, 1.2)),
        ("C-mixed-1", (0.5, -0.4, 0.3, -0.2, 0.6, -0.7)),
        ("C-mixed-2", (-1.0, 0.8, -0.5, 1.1, -0.3, 0.9)),
        ("D-near-upper", (near_full, near_full, near_elbow, near_full, near_full, near_full)),
        ("D-near-lower", (-near_full, -near_full, -near_elbow, -near_full, -near_full, -near_full)),
        ("E-fixed-1", (0.1, -0.2, 0.3, -0.4, 0.5, -0.6)),
        ("E-fixed-2", (math.pi / 2.0, -math.pi / 2.0, math.pi / 2.0, -math.pi / 4.0, math.pi / 6.0, -math.pi / 3.0)),
        ("E-fixed-3", (0.01, 0.02, -0.03, 0.04, -0.05, 0.06)),
        ("E-fixed-4", (2.0, -1.5, 1.0, -2.0, 1.5, -1.0)),
    )
    return tuple({"test_id": test_id, "joint_vector_rad": joints} for test_id, joints in rows)
