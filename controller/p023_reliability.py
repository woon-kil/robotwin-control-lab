"""P-023 reliability accounting.

ACK는 completion이 아니다.
이 모듈은 ServoCtrl write와 #1J/를 만들지 않는다.
100 cycle은 industrial reliability가 아니다.
"""

from __future__ import annotations

import math

from digital_twin.controller_axis import MAPPING_BOUNDARY, normalize_motor_snapshot

NOT_AVAILABLE = "NOT_AVAILABLE"
RING = (1, 2, 3)
MOTION_NAMES = ("Motor[1].JogPos", "Motor[1].DesPos", "Motor[1].Pos", "Motor[1].ActPos")
FAULT_NAMES = ("Motor[1].FeFatal", "Motor[1].AmpFault", "Motor[1].LimitStop")
_CODES = {
    "REL-1": "FULL_RELIABILITY_PASS",
    "REL-2": "MOTION_RELIABILITY_FAILURE",
    "REL-3": "RECONNECT_RELIABILITY_FAILURE",
    "REL-4": "CONTROLLER_STATE_FAILURE",
    "REL-5": "TRANSPORT_FAILURE",
    "REL-6": "SIMULATOR_RUNTIME_FAILURE",
    "REL-7": "EVIDENCE_OR_REGRESSION_FAILURE",
}


def alternate_targets(position: str, count: int = 100) -> list[int] | None:
    number = _number(position)
    if number is None or number != int(number) or count < 1:
        return None
    current = int(number)
    targets: list[int] = []
    for _ in range(count):
        target = next_target(current)
        if target == current:
            return None
        targets.append(target)
        current = target
    if any(left == right for left, right in zip(targets, targets[1:])):
        return None
    return targets


def next_target(current: int) -> int:
    if current in RING:
        return RING[(RING.index(current) + 1) % len(RING)]
    return RING[0]


def reconnect_pair(position: int) -> tuple[int, int]:
    blocked = next_target(position)
    renewed = next_target(blocked)
    return blocked, renewed


def motion_completed(sample: dict[str, str], target: str, ack_present: bool) -> bool:
    del ack_present
    return all(_equal(sample.get(name, NOT_AVAILABLE), target) for name in MOTION_NAMES)


def evaluate_observation(
    post: dict[str, str],
    *,
    target: str,
    ack_present: bool,
    error_present: bool,
    fresh: bool,
    communication_state: str,
    observed_at: str,
) -> dict[str, object]:
    """POST snapshot만으로 cycle 결과를 만든다. activation command는 없다."""
    del ack_present
    observation = normalize_motor_snapshot(
        post,
        communication_state=communication_state,
        command_target=_number(target),
        observed_at=observed_at,
        fresh=fresh,
    )
    positions_ok = motion_completed(post, target, False)
    faults_ok = all(_equal(post.get(name, NOT_AVAILABLE), "0") for name in FAULT_NAMES)
    axis_held = _equal(post.get("Motor[1].ServoCtrl", NOT_AVAILABLE), "1") and _equal(post.get("Motor[1].ClosedLoop", NOT_AVAILABLE), "1")
    completed = bool(
        positions_ok
        and faults_ok
        and axis_held
        and observation.valid
        and observation.tracking_success
        and observation.tracking_error == 0
        and not error_present
    )
    event = "COMPLETED" if completed else _failure_event(post, observation.invalid_reason, fresh, communication_state)
    return {
        "event": event,
        "completed": completed,
        "stop": stop_decision(event),
        "tracking_error": observation.tracking_error,
        "observation": observation.to_dict(),
    }


def stop_decision(event: str) -> bool:
    return event in {
        "TIMEOUT",
        "DISCONNECT",
        "PROTOCOL_ERROR",
        "FAULT",
        "MALFORMED",
        "NONFINITE",
        "CLOSED_LOOP_LOST",
        "SERVO_LOST",
        "RUNTIME",
    }


def apply_event(counters: dict[str, int], event: str) -> dict[str, int]:
    updated = dict(counters)
    updated["attempted_cycles"] = updated.get("attempted_cycles", 0) + 1
    if event == "COMPLETED":
        updated["completed_cycles"] = updated.get("completed_cycles", 0) + 1
    else:
        updated["failed_cycles"] = updated.get("failed_cycles", 0) + 1
    field = {
        "TIMEOUT": "timeout_count",
        "DISCONNECT": "disconnect_count",
        "PROTOCOL_ERROR": "protocol_error_count",
        "FAULT": "fault_count",
        "NONFINITE": "nonfinite_count",
        "MALFORMED": "malformed_count",
        "TRACKING_MISMATCH": "tracking_mismatch_count",
        "STALE": "stale_observation_count",
        "UNEXPECTED": "unexpected_state_count",
        "CLOSED_LOOP_LOST": "unexpected_state_count",
        "SERVO_LOST": "unexpected_state_count",
        "RUNTIME": "runtime_failure_count",
    }.get(event)
    if field is not None:
        updated[field] = updated.get(field, 0) + 1
    return updated


def state_leakage(previous_post: dict[str, str] | None, pre: dict[str, str]) -> bool:
    if previous_post is None:
        return False
    return not _equal(previous_post.get("Motor[1].ActPos", NOT_AVAILABLE), pre.get("Motor[1].ActPos", NOT_AVAILABLE))


def disconnect_blocked(*, result: str, error_code: str | None, command_sent: bool, jog_observed: bool) -> bool:
    return result == "REJECTED" and error_code == "DISCONNECTED" and command_sent is False and jog_observed is False


def reconnect_plan(*, fresh_read: bool, ready: bool) -> dict[str, object]:
    if not fresh_read:
        return {"motion": False, "activation": [], "reason": "FRESH_READ_REQUIRED"}
    if not ready:
        return {"motion": False, "activation": [], "reason": "RECONNECT_NOT_READY"}
    return {"motion": True, "activation": [], "reason": "NEW_MOTION"}


def percentile(values: list[float], percent: float) -> float | None:
    """정렬 표본의 linear interpolation. servo cycle time이 아니다."""
    if not values:
        return None
    if percent < 0 or percent > 100:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (percent / 100.0) * (len(ordered) - 1)
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def metric_summary(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"mean": None, "p50": None, "p95": None, "p99": None, "max": None}
    return {
        "mean": sum(values) / len(values),
        "p50": percentile(values, 50),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": max(values),
    }


def classify_reliability(
    *,
    preflight_transport: bool,
    preflight_ready: bool,
    motion_attempted: int,
    motion_completed: int,
    motion_defect_count: int,
    reconnect_ran: bool,
    reconnect_acceptance: bool,
    controller_stop: bool,
    transport_stop: bool,
    runtime_failure: bool,
    final_ready: bool,
    negative_pass: bool,
    domain_ok: bool,
    forbidden_write: bool,
    regression_passed: bool | None,
    evidence_complete: bool,
) -> dict[str, object]:
    """REL-1도 P-023 FINAL PASS 선언이 아니다."""
    if (not evidence_complete) or (not domain_ok) or forbidden_write or regression_passed is False or not negative_pass:
        return _result("REL-7", regression_passed, False)
    if runtime_failure:
        return _result("REL-6", regression_passed, False)
    if (not preflight_transport) or transport_stop:
        return _result("REL-5", regression_passed, False)
    if (not preflight_ready) or controller_stop or not final_ready:
        return _result("REL-4", regression_passed, False)
    motion_ok = motion_attempted == 100 and motion_completed == 100 and motion_defect_count == 0
    if not motion_ok:
        return _result("REL-2", regression_passed, False)
    if not reconnect_ran or not reconnect_acceptance:
        return _result("REL-3", regression_passed, False)
    return _result("REL-1", regression_passed, regression_passed is True)


def empty_counters() -> dict[str, int]:
    return {
        "attempted_cycles": 0,
        "completed_cycles": 0,
        "failed_cycles": 0,
        "timeout_count": 0,
        "disconnect_count": 0,
        "protocol_error_count": 0,
        "fault_count": 0,
        "nonfinite_count": 0,
        "malformed_count": 0,
        "tracking_mismatch_count": 0,
        "stale_observation_count": 0,
        "unexpected_state_count": 0,
        "runtime_failure_count": 0,
        "state_leakage_or_external_change": 0,
    }


def _failure_event(post: dict[str, str], invalid_reason: str | None, fresh: bool, communication_state: str) -> str:
    if communication_state != "HEALTHY":
        return "DISCONNECT"
    if not fresh:
        return "STALE"
    if invalid_reason == "MALFORMED":
        if _nonfinite(post.get("Motor[1].ActPos")) or _nonfinite(post.get("Motor[1].DesPos")):
            return "NONFINITE"
        return "MALFORMED"
    if invalid_reason == "INCOMPLETE":
        return "PROTOCOL_ERROR"
    if _number(post.get("Motor[1].ServoCtrl", NOT_AVAILABLE)) != 1:
        return "SERVO_LOST"
    if _number(post.get("Motor[1].ClosedLoop", NOT_AVAILABLE)) != 1:
        return "CLOSED_LOOP_LOST"
    for name in FAULT_NAMES:
        number = _number(post.get(name, NOT_AVAILABLE))
        if number is None:
            return "PROTOCOL_ERROR"
        if number != 0:
            return "FAULT"
    if not motion_completed(post, post.get("Motor[1].JogPos", NOT_AVAILABLE), False):
        return "TRACKING_MISMATCH"
    return "TRACKING_MISMATCH"


def _result(code: str, regression_passed: bool | None, acceptance_complete: bool) -> dict[str, object]:
    return {
        "classification": code,
        "classification_name": _CODES[code],
        "confidence": "MEDIUM" if code == "REL-1" and acceptance_complete else "LOW",
        "confirmed": False,
        "p023_final_pass_declared": False,
        "root_cause": "NOT_CONFIRMED",
        "regression_passed": regression_passed,
        "acceptance_complete": acceptance_complete,
        "industrial_reliability_claimed": False,
    }


def _equal(left: str, right: str) -> bool:
    if left == NOT_AVAILABLE or right == NOT_AVAILABLE:
        return False
    if left == right:
        return True
    left_number = _number(left)
    right_number = _number(right)
    if left_number is None or right_number is None:
        return False
    return left_number == right_number


def _number(value: str | None) -> float | None:
    if value is None or value == NOT_AVAILABLE:
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    if not math.isfinite(number):
        return None
    return number


def _nonfinite(value: str | None) -> bool:
    if value is None:
        return False
    try:
        number = float(value)
    except ValueError:
        return False
    return not math.isfinite(number)
