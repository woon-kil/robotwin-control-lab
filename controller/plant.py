"""Deterministic joint plant.

DETERMINISTIC PROJECT TEST MODEL.
NOT ACTUAL UR10e SERVO DYNAMICS.
NOT ACTUAL PMAC SERVO DYNAMICS.
"""

from __future__ import annotations


def slew_step(
    actual: tuple[float, ...],
    target: tuple[float, ...],
    max_velocity_rad_s: tuple[float, ...],
    dt_s: float,
) -> tuple[float, ...]:
    """목표까지 한 step. 남은 거리가 속도 한도 안이면 그 목표에 닿는다."""
    updated: list[float] = []
    for current, goal, max_velocity in zip(actual, target, max_velocity_rad_s, strict=True):
        delta = goal - current
        limit = max_velocity * dt_s
        if delta > limit:
            delta = limit
        elif delta < -limit:
            delta = -limit
        updated.append(current + delta)
    return tuple(updated)
