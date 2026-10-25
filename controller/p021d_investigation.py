"""P-021D #1J/ reactivation causal helpers.

ACK는 motion completion이 아니다.
정상 probe에서 #1J/를 보내지 않는다.
stall을 만드는 write는 이 모듈에 없다.
"""

from __future__ import annotations

NOT_AVAILABLE = "NOT_AVAILABLE"

TIMELINE_NAMES = (
    "Motor[1].JogPos",
    "Motor[1].DesPos",
    "Motor[1].DesVel",
    "Motor[1].Desired.Pos",
    "Motor[1].Desired.Vel",
    "Motor[1].Desired.TimerEnabled",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].ActVel",
    "Motor[1].InPos",
)

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


def is_rb1(sample: dict[str, str], target: str) -> bool:
    return (
        values_equal(sample.get("Motor[1].JogPos", NOT_AVAILABLE), target)
        and not values_equal(sample.get("Motor[1].DesPos", NOT_AVAILABLE), target)
        and not values_equal(sample.get("Motor[1].Pos", NOT_AVAILABLE), target)
        and not values_equal(sample.get("Motor[1].ActPos", NOT_AVAILABLE), target)
        and values_equal(sample.get("Motor[1].DesVel", NOT_AVAILABLE), "0")
    )


def final_observation(samples: list[dict[str, str]], target: str) -> str:
    if not samples:
        return "OTHER"
    last = samples[-1]
    if motion_completed(last, target, True):
        return "COMPLETED"
    if is_rb1(last, target):
        return "RB-1"
    return "OTHER"


def activation_flags(snapshot: dict[str, str]) -> dict[str, bool | None]:
    servo = _number(snapshot.get("Motor[1].ServoCtrl", NOT_AVAILABLE))
    closed = _number(snapshot.get("Motor[1].ClosedLoop", NOT_AVAILABLE))
    faults = [_number(snapshot.get(name, NOT_AVAILABLE)) for name in _FAULTS]
    if servo is None or closed is None or any(item is None for item in faults):
        return {"servo_ok": None, "closed_loop_ok": None, "fault_present": None, "complete": False}
    return {
        "servo_ok": servo == 1,
        "closed_loop_ok": closed == 1,
        "fault_present": any(item != 0 for item in faults),
        "complete": True,
    }


def next_probe_target(position: str, jog_position: str, previous: int | None) -> int | None:
    pos = _number(position)
    jog = _number(jog_position)
    if pos is None or jog is None:
        return None
    blocked = {pos, jog}
    if previous is None:
        order = (2, 1, 0, 3, 4)
    elif previous == 2:
        order = (1, 0, 3, 4, 5)
    elif previous == 1:
        order = (2, 0, 3, 4, 5)
    else:
        order = (0, 1, 2, 3, 4, 5)
    for candidate in order:
        if candidate not in blocked:
            return candidate
    return None


def choose_recovery_target(position: str, jog_position: str, failed_target: int) -> int | None:
    pos = _number(position)
    jog = _number(jog_position)
    if pos is None or jog is None:
        return None
    blocked = {pos, jog, float(failed_target)}
    for candidate in (3, 0, 1, 2, 4, 5):
        if candidate not in blocked:
            return candidate
    return None


def classify_reactivation(
    *,
    evidence_complete: bool,
    unexpected_write: bool,
    restart_performed: bool,
    probe_results: tuple[str, ...],
    stall_servo_ok: bool | None,
    stall_closed_loop_ok: bool | None,
    stall_fault_present: bool | None,
    intervention: str | None,
    recovery_result: str | None,
    cycles: int,
) -> dict[str, object]:
    """한 번의 #1J/ 연관을 P-021 recovery semantics로 확정하지 않는다."""
    if not evidence_complete or unexpected_write or restart_performed:
        return _result("RD-6", "EXPERIMENT_INVALID", "LOW", "causal isolation did not hold")
    if any(result == "OTHER" for result in probe_results) or recovery_result == "OTHER":
        return _result("RD-6", "EXPERIMENT_INVALID", "LOW", "causal isolation did not hold")
    if any(result == "RB-1" for result in probe_results):
        if stall_servo_ok is not True or stall_closed_loop_ok is not True:
            return _result("RD-3", "ACTIVATION_STATE_LOST", "MEDIUM", "activation state was not the #1J/ only precondition")
        if stall_fault_present is True or intervention not in {None, "#1J/"}:
            return _result("RD-6", "EXPERIMENT_INVALID", "LOW", "causal isolation did not hold")
        if intervention is None:
            if "COMPLETED" in probe_results:
                return _result("RD-5", "INTERMITTENT", "LOW", "outcomes were not stable under the same condition")
            return _result("RD-6", "EXPERIMENT_INVALID", "LOW", "causal isolation did not hold")
        if recovery_result == "COMPLETED":
            confidence = "HIGH" if cycles >= 2 else "MEDIUM"
            return _result("RD-1", "REACTIVATION_RECOVERS_PIPELINE", confidence, "#1J/ reactivation is causally associated with restoring motion generation")
        if recovery_result == "RB-1":
            return _result("RD-2", "REACTIVATION_DOES_NOT_RECOVER", "MEDIUM", "#1J/ alone hypothesis weakened")
        return _result("RD-6", "EXPERIMENT_INVALID", "LOW", "causal isolation did not hold")
    if intervention is not None:
        return _result("RD-6", "EXPERIMENT_INVALID", "LOW", "causal isolation did not hold")
    if len(probe_results) >= 3 and all(result == "COMPLETED" for result in probe_results):
        return _result("RD-4", "NO_STALL_OBSERVED", "MEDIUM", "P021D_WAITING_FOR_STALL", "P021D_WAITING_FOR_STALL")
    return _result("RD-6", "EXPERIMENT_INVALID", "LOW", "causal isolation did not hold")


def _result(code: str, name: str, confidence: str, claim: str, operational: str | None = None) -> dict[str, object]:
    result: dict[str, object] = {
        "classification": code,
        "classification_name": name,
        "confidence": confidence,
        "causal_claim": claim,
        "confirmed": False,
    }
    if operational is not None:
        result["operational_state"] = operational
    return result


def _number(value: str) -> float | None:
    if value == NOT_AVAILABLE:
        return None
    try:
        return float(value)
    except ValueError:
        return None
