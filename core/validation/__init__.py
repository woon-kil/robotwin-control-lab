"""Cross-simulator TCP comparison. 시뮬레이터 runtime을 import하지 않는다."""

from core.validation.tcp_compare import (
    orientation_error_rad,
    position_error_m,
    unity_pose_to_canonical,
)

__all__ = ["orientation_error_rad", "position_error_m", "unity_pose_to_canonical"]
