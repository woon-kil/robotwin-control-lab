"""P-021 final live recovery classification.

ACK는 motion completion이 아니다.
connect 성공은 recovery completion이 아니다.
JogPos만 target인 결과는 COMPLETED가 아니다.
"""

from __future__ import annotations

NOT_AVAILABLE = "NOT_AVAILABLE"
_MOTION = ("Motor[1].DesPos", "Motor[1].Pos", "Motor[1].ActPos")
_FAULTS = ("Motor[1].FeFatal", "Motor[1].AmpFault", "Motor[1].LimitStop")


def values_equal(left: str, right: str) -> bool:
    if left == NOT_AVAILABLE or right == NOT_AVAILABLE:
        return False
    if left == right:
        return True
    left_number = _number(left)
    right_number = _number(right)
    if left_number is None or right_number is None:
        return False
    return left_number == right_number


def motion_completed(sample: dict[str, str], target: str, ack_present: bool) -> bool:
    del ack_present
    return all(values_equal(sample.get(name, NOT_AVAILABLE), target) for name in _MOTION)


def is_rb1(before: dict[str, str], after: dict[str, str], target: str) -> bool:
    return (
        values_equal(after.get("Motor[1].JogPos", NOT_AVAILABLE), target)
        and values_equal(after.get("Motor[1].DesPos", NOT_AVAILABLE), before.get("Motor[1].DesPos", NOT_AVAILABLE))
        and values_equal(after.get("Motor[1].Pos", NOT_AVAILABLE), before.get("Motor[1].Pos", NOT_AVAILABLE))
        and values_equal(after.get("Motor[1].ActPos", NOT_AVAILABLE), before.get("Motor[1].ActPos", NOT_AVAILABLE))
        and not values_equal(after.get("Motor[1].DesPos", NOT_AVAILABLE), target)
        and values_equal(after.get("Motor[1].DesVel", NOT_AVAILABLE), "0")
    )


def readiness_of(snapshot: dict[str, str]) -> str:
    servo = _number(snapshot.get("Motor[1].ServoCtrl", NOT_AVAILABLE))
    closed = _number(snapshot.get("Motor[1].ClosedLoop", NOT_AVAILABLE))
    faults = [_number(snapshot.get(name, NOT_AVAILABLE)) for name in _FAULTS]
    if servo is None or closed is None or any(item is None for item in faults):
        return "INCOMPLETE"
    if any(item != 0 for item in faults):
        return "FAULT_PRESENT"
    if servo <= 0 or closed != 1:
        return "NOT_READY"
    return "RECOVERED"


def choose_precondition_target(position: str, jog_position: str) -> int | None:
    return _first_free((1, 0, 3, 2, 4), position, jog_position)


def choose_blocked_target(precondition_target: int) -> int:
    for candidate in (2, 0, 3, 1, 4, 5):
        if candidate != precondition_target:
            return candidate
    return 5


def choose_new_target(position: str, jog_position: str, precondition_target: int, blocked_target: int) -> int | None:
    pos = _number(position)
    jog = _number(jog_position)
    if pos is None or jog is None:
        return None
    blocked = {pos, jog, float(precondition_target), float(blocked_target)}
    for candidate in (3, 0, 1, 2, 4, 5):
        if candidate not in blocked:
            return candidate
    return None


def classify_final_lifecycle(
    *,
    evidence_complete: bool,
    unexpected_write: bool,
    restart_performed: bool,
    blind_reactivation: bool,
    precondition_ready: bool,
    precondition_completed: bool | None,
    disconnect_verified: bool,
    blocked_result: str | None,
    blocked_error: str | None,
    command_sent: bool | None,
    simulator_alive: bool,
    reconnect_healthy: bool,
    fresh_read: bool,
    readiness: str | None,
    new_target_distinct: bool,
    motion_attempted: bool,
    motion_accepted: bool,
    motion_completed_value: bool | None,
    rb1: bool | None,
    faults_clear: bool | None,
) -> dict[str, object]:
    """한 번의 live cycle을 P-021 FINAL PASS로 선언하지 않는다."""
    if not evidence_complete or unexpected_write or restart_performed or blind_reactivation:
        return _result("PF-7", "EXPERIMENT_INVALID", "causal isolation did not hold")
    if not precondition_ready:
        return _result("PF-7", "EXPERIMENT_INVALID", "precondition state was not motion-ready")
    if precondition_completed is not True:
        return _result("PRECONDITION_MOTION_FAILED", "PRECONDITION_MOTION_FAILED", "disconnect test was not started")
    blocked_ok = disconnect_verified and blocked_result == "REJECTED" and blocked_error == "DISCONNECTED" and command_sent is False
    if not blocked_ok:
        return _result("PF-2", "DISCONNECT_BLOCK_FAILED", "disconnected motion reached the transport")
    if not simulator_alive:
        return _result("PF-7", "EXPERIMENT_INVALID", "simulator liveness was not observed")
    if not reconnect_healthy:
        return _result("PF-3", "RECONNECT_FAILED", "adapter reconnect did not become HEALTHY")
    if not fresh_read:
        return _result("PF-7", "EXPERIMENT_INVALID", "fresh Motor[1] read-back was required")
    if readiness != "RECOVERED":
        return _result("PF-4", "NOT_READY_AFTER_RECONNECT", "fresh read did not verify motion readiness")
    if not new_target_distinct or not motion_attempted:
        return _result("PF-7", "EXPERIMENT_INVALID", "new motion target was not isolated")
    if rb1 is True:
        return _result("PF-5", "POST_RECOVERY_RB1", "JogPos changed without DesPos Pos and ActPos")
    if motion_accepted is not True or motion_completed_value is not True or faults_clear is not True:
        return _result("PF-6", "POST_RECOVERY_OTHER_MOTION_FAILURE", "post-recovery motion did not reach the target")
    return _result("PF-1", "FULL_RECOVERY_PASS", "live adapter recovery lifecycle completed on this run")


def _result(code: str, name: str, claim: str) -> dict[str, object]:
    return {
        "classification": code,
        "classification_name": name,
        "confidence": "MEDIUM" if code == "PF-1" else "LOW",
        "causal_claim": claim,
        "confirmed": False,
        "final_pass_declared": False,
    }


def _first_free(order: tuple[int, ...], position: str, jog_position: str) -> int | None:
    pos = _number(position)
    jog = _number(jog_position)
    if pos is None or jog is None:
        return None
    blocked = {pos, jog}
    for candidate in order:
        if candidate not in blocked:
            return candidate
    return None


def _number(value: str) -> float | None:
    if value == NOT_AVAILABLE:
        return None
    try:
        return float(value)
    except ValueError:
        return None
