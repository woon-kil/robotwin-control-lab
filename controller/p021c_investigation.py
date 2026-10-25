"""P-021C session-reset comparison helpers.

Transport reconnect는 Simulator restart가 아니다.
ACK는 motion completion이 아니다.
"""

from __future__ import annotations

NOT_AVAILABLE = "NOT_AVAILABLE"

SNAPSHOT_NAMES = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].DesPos",
    "Motor[1].DesVel",
    "Motor[1].DesVelZero",
    "Motor[1].JogPos",
    "Motor[1].ProgJogPos",
    "Motor[1].JogVel",
    "Motor[1].JogSpeed",
    "Motor[1].JogTa",
    "Motor[1].JogTs",
    "Motor[1].MaxSpeed",
    "Motor[1].Desired.Pos",
    "Motor[1].Desired.Vel",
    "Motor[1].Desired.TimerEnabled",
    "Motor[1].AmpEna",
    "Motor[1].AmpFault",
    "Motor[1].FeFatal",
    "Motor[1].FeWarn",
    "Motor[1].LimitStop",
    "Motor[1].PlusLimit",
    "Motor[1].MinusLimit",
    "Motor[1].MaxPos",
    "Motor[1].MinPos",
    "Motor[1].SoftLimit",
    "Motor[1].SoftPlusLimit",
    "Motor[1].SoftMinusLimit",
    "Motor[1].SoftLimitDir",
    "Motor[1].InPos",
    "Motor[1].Coord",
    "Motor[1].Ctrl",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
    "Coord[0].ProgRunning",
)

TIMELINE_NAMES = (
    "Motor[1].JogPos",
    "Motor[1].DesPos",
    "Motor[1].DesVel",
    "Motor[1].DesVelZero",
    "Motor[1].Desired.Pos",
    "Motor[1].Desired.Vel",
    "Motor[1].Desired.TimerEnabled",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].ActVel",
    "Motor[1].InPos",
)


def values_equal(left: str, right: str) -> bool:
    if left == NOT_AVAILABLE or right == NOT_AVAILABLE:
        return False
    if left == right:
        return True
    try:
        return float(left) == float(right)
    except ValueError:
        return False


def readiness(snapshot: dict[str, str]) -> str:
    """Read-back만 본다. ServoCtrl write나 #1J/를 만들지 않는다."""
    servo = _number(snapshot.get("Motor[1].ServoCtrl", NOT_AVAILABLE))
    closed = _number(snapshot.get("Motor[1].ClosedLoop", NOT_AVAILABLE))
    fatal = _number(snapshot.get("Motor[1].FeFatal", NOT_AVAILABLE))
    amp = _number(snapshot.get("Motor[1].AmpFault", NOT_AVAILABLE))
    limit = _number(snapshot.get("Motor[1].LimitStop", NOT_AVAILABLE))
    if None in {servo, closed, fatal, amp, limit}:
        return "NOT_READY"
    if fatal != 0 or amp != 0 or limit != 0:
        return "FAULT_PRESENT"
    if servo <= 0 or closed != 1:
        return "NOT_READY"
    return "RECOVERED"


def choose_target(position: str, jog_position: str) -> int:
    position_number = _number(position)
    jog_number = _number(jog_position)
    for candidate in (0, 1, 2, 3, 4):
        if candidate != position_number and candidate != jog_number:
            return candidate
    return 5


def motion_completed(sample: dict[str, str], target: str, ack_present: bool) -> bool:
    del ack_present
    return all(values_equal(sample.get(name, NOT_AVAILABLE), target) for name in ("Motor[1].DesPos", "Motor[1].Pos", "Motor[1].ActPos"))


def restart_diff(before: dict[str, str], after: dict[str, str]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for name in SNAPSHOT_NAMES:
        left = before.get(name, NOT_AVAILABLE)
        right = after.get(name, NOT_AVAILABLE)
        if left == NOT_AVAILABLE or right == NOT_AVAILABLE:
            tag = NOT_AVAILABLE
        elif values_equal(left, right):
            tag = "UNCHANGED"
        elif _number(right) == 0 and _number(left) not in {None, 0}:
            tag = "RESET_TO_ZERO"
        else:
            tag = "CHANGED"
        rows.append({"variable": name, "before": left, "after": right, "tag": tag})
    return rows


def classify_session_reset(
    *,
    before_completed: bool | None,
    after_ran: bool,
    after_completed: bool | None,
    activation_required: bool,
    boot_ok: bool,
    snapshot_ok: bool,
) -> dict[str, object]:
    """한 번의 restart 상관을 root cause로 확정하지 않는다."""
    if not after_ran:
        return {
            "classification": "WAITING_FOR_MANUAL_RESTART",
            "confidence": "LOW",
            "causal_claim": "not_run",
            "confirmed": False,
        }
    if not boot_ok or not snapshot_ok or before_completed is None or after_completed is None:
        return _result("RC-S5", "EXPERIMENT_INVALID", "LOW", "experiment_invalid", False)
    if (not before_completed) and after_completed and activation_required:
        return _result("RC-S3", "RESTART_CHANGES_ACTIVATION_STATE", "LOW", "FOLLOW-UP CAUSAL TEST REQUIRED", False)
    if (not before_completed) and after_completed:
        return _result("RC-S1", "SESSION_RESET_RECOVERS_MOTION", "MEDIUM", "SIMULATOR_SESSION_STATE_CANDIDATE", False)
    if (not before_completed) and (not after_completed):
        return _result("RC-S2", "SESSION_RESET_DOES_NOT_RECOVER", "MEDIUM", "session residue hypothesis weakened", False)
    if before_completed != after_completed:
        return _result("RC-S4", "INTERMITTENT", "LOW", "not_confirmed", False)
    return _result("RC-S5", "EXPERIMENT_INVALID", "LOW", "stall was not present before restart", False)


def _result(code: str, name: str, confidence: str, claim: str, confirmed: bool) -> dict[str, object]:
    return {
        "classification": code,
        "classification_name": name,
        "confidence": confidence,
        "causal_claim": claim,
        "confirmed": confirmed,
    }


def _number(value: str) -> float | None:
    if value == NOT_AVAILABLE:
        return None
    try:
        return float(value)
    except ValueError:
        return None
