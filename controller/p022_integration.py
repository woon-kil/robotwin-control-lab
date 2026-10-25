"""P-022 controller-domain integration decisions.

ACK는 motion completion이 아니다.
ServoCtrl write와 #1J/는 이 integration의 recovery가 아니다.
"""

from __future__ import annotations

NOT_AVAILABLE = "NOT_AVAILABLE"
MOTION_NAMES = ("Motor[1].DesPos", "Motor[1].Pos", "Motor[1].ActPos")
_CODES = {
    "DT-1": "FULL_INTEGRATION_PASS",
    "DT-2": "LIVE_MOTION_FAILED",
    "DT-3": "OBSERVATION_BRIDGE_FAILED",
    "DT-4": "DOMAIN_BOUNDARY_VIOLATION",
    "DT-5": "FAIL_CLOSED_VIOLATION",
    "DT-6": "REGRESSION",
}


def choose_cycle_targets(position: str) -> tuple[int, int, int] | None:
    number = _number(position)
    if number is None or number != int(number):
        return None
    current = int(number)
    if current in (0, 1, 2, 3):
        first = (current + 1) % 4
        second = (first + 1) % 4
        return (first, second, current)
    pool = [value for value in (0, 1, 2, 3) if value != current]
    if len(pool) < 3:
        return None
    return (pool[0], pool[1], pool[2])


def motion_completed(sample: dict[str, str], target: str, ack_present: bool) -> bool:
    del ack_present
    return all(values_equal(sample.get(name, NOT_AVAILABLE), target) for name in MOTION_NAMES)


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


def precondition_ready(snapshot: dict[str, str]) -> bool:
    servo = _number(snapshot.get("Motor[1].ServoCtrl", NOT_AVAILABLE))
    closed = _number(snapshot.get("Motor[1].ClosedLoop", NOT_AVAILABLE))
    if servo != 1 or closed != 1:
        return False
    for name in ("Motor[1].FeFatal", "Motor[1].AmpFault", "Motor[1].LimitStop"):
        if _number(snapshot.get(name, NOT_AVAILABLE)) != 0:
            return False
    return True


def classify_integration(
    *,
    precondition_ready: bool,
    cycles_completed: int,
    observation_ok: bool,
    domain_violation: bool,
    fail_closed_ok: bool,
    regression_passed: bool | None,
    forbidden_write: bool,
) -> dict[str, object]:
    """DT-1도 P-022 FINAL PASS가 아니다."""
    if domain_violation:
        return _result("DT-4", regression_passed, False)
    if not fail_closed_ok:
        return _result("DT-5", regression_passed, False)
    if regression_passed is False:
        return _result("DT-6", regression_passed, False)
    live_ok = precondition_ready and cycles_completed == 3 and not forbidden_write
    if not live_ok:
        return _result("DT-2", regression_passed, False)
    if not observation_ok:
        return _result("DT-3", regression_passed, False)
    return _result("DT-1", regression_passed, regression_passed is True)


def _result(code: str, regression_passed: bool | None, acceptance_complete: bool) -> dict[str, object]:
    return {
        "classification": code,
        "classification_name": _CODES[code],
        "confidence": "MEDIUM" if code == "DT-1" and acceptance_complete else "LOW",
        "confirmed": False,
        "p022_final_pass_declared": False,
        "regression_passed": regression_passed,
        "acceptance_complete": acceptance_complete,
    }


def _number(value: str) -> float | None:
    if value == NOT_AVAILABLE:
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number
