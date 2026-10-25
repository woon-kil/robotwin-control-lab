"""P-022 clean-restart baseline decisions.

S0를 읽기 전에는 motor write를 만들지 않는다.
ACK는 motion completion이 아니다.
Xenomai 메시지의 root cause는 확정하지 않는다.
"""

from __future__ import annotations

NOT_AVAILABLE = "NOT_AVAILABLE"

SNAPSHOT_NAMES = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].AmpEna",
    "Motor[1].JogPos",
    "Motor[1].DesPos",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].DesVel",
    "Motor[1].DesVelZero",
    "Motor[1].InPos",
    "Motor[1].FeFatal",
    "Motor[1].FeWarn",
    "Motor[1].AmpFault",
    "Motor[1].LimitStop",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
)

MOTION_NAMES = (
    "Motor[1].JogPos",
    "Motor[1].DesPos",
    "Motor[1].Pos",
    "Motor[1].ActPos",
)

FAULT_NAMES = (
    "Motor[1].FeFatal",
    "Motor[1].AmpFault",
    "Motor[1].LimitStop",
)

_CODES = {
    "BR-1": "BASELINE_READY",
    "BR-2": "TRANSPORT_FAILED",
    "BR-3": "GPASCII_FAILED",
    "BR-4": "FAULT_PRESENT",
    "BR-5": "ACTIVATION_FAILED",
    "BR-6": "MOTION_FAILED",
    "BR-7": "EXPERIMENT_INVALID",
}


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


def transport_ok(hostname: str, uname: str, eth0: str) -> bool:
    return hostname.strip() == "ppmac" and uname.strip() == "Linux" and "172.20.0.200" in eth0


def gpascii_ok(value: str) -> bool:
    return _number(value) is not None


def snapshot_complete(snapshot: dict[str, str]) -> bool:
    return all(snapshot.get(name, NOT_AVAILABLE) != NOT_AVAILABLE for name in SNAPSHOT_NAMES)


def faults_clear(snapshot: dict[str, str]) -> bool:
    return all(values_equal(snapshot.get(name, NOT_AVAILABLE), "0") for name in FAULT_NAMES)


def ready_state(snapshot: dict[str, str]) -> bool:
    return (
        values_equal(snapshot.get("Motor[1].ServoCtrl", NOT_AVAILABLE), "1")
        and values_equal(snapshot.get("Motor[1].ClosedLoop", NOT_AVAILABLE), "1")
        and faults_clear(snapshot)
    )


def activation_decision(snapshot: dict[str, str] | None) -> dict[str, str | None]:
    """다음에 허용되는 activation 명령은 최대 하나다. jog는 여기서 만들지 않는다."""
    if snapshot is None or not snapshot_complete(snapshot):
        return {"status": "INCOMPLETE", "command": None}
    if not faults_clear(snapshot):
        return {"status": "FAULT", "command": None}
    servo = _number(snapshot.get("Motor[1].ServoCtrl", NOT_AVAILABLE))
    closed = _number(snapshot.get("Motor[1].ClosedLoop", NOT_AVAILABLE))
    if servo is None or closed is None:
        return {"status": "INCOMPLETE", "command": None}
    if servo == 0:
        return {"status": "SERVO", "command": "Motor[1].ServoCtrl=1"}
    if servo != 1:
        return {"status": "UNEXPECTED_SERVO", "command": None}
    if closed != 1:
        return {"status": "CLOSE", "command": "#1J/"}
    return {"status": "READY", "command": None}


def readback_ok(command: str, value: str) -> bool:
    if command == "Motor[1].ServoCtrl=1":
        return values_equal(value, "1")
    if command == "#1J/":
        return values_equal(value, "1")
    return False


def allowed_writes(*, transport_ok: bool, snapshot: dict[str, str] | None, readback_failed: bool) -> list[str]:
    if not transport_ok or snapshot is None or readback_failed:
        return []
    command = activation_decision(snapshot)["command"]
    if command is None:
        return []
    return [command]


def choose_probe_target(position: str, jog_position: str) -> int | None:
    position_number = _number(position)
    jog_number = _number(jog_position)
    if position_number is None or jog_number is None:
        return None
    for candidate in (1, 0, 2, 3, 4):
        if candidate != position_number and candidate != jog_number:
            return candidate
    return 5


def motion_command(snapshot: dict[str, str] | None, *, ready: bool) -> str | None:
    if not ready or snapshot is None:
        return None
    target = choose_probe_target(
        snapshot.get("Motor[1].Pos", NOT_AVAILABLE),
        snapshot.get("Motor[1].JogPos", NOT_AVAILABLE),
    )
    if target is None:
        return None
    return "#1j=" + str(target)


def motion_completed(sample: dict[str, str], target: str, ack_present: bool) -> bool:
    del ack_present
    return all(values_equal(sample.get(name, NOT_AVAILABLE), target) for name in MOTION_NAMES)


def classify_baseline(
    *,
    transport_ok: bool,
    gpascii_ok: bool,
    snapshot_recorded: bool,
    snapshot_complete: bool,
    writes_before_snapshot: bool,
    fault_present: bool,
    activation_required: bool,
    activation_readback_ok: bool,
    ready: bool,
    motion_completed: bool | None,
    unexpected_write: bool,
) -> dict[str, object]:
    """BR-1도 P-022 FINAL PASS가 아니다."""
    if writes_before_snapshot or unexpected_write or (snapshot_recorded and not snapshot_complete):
        return _result("BR-7")
    if not transport_ok:
        return _result("BR-2")
    if not gpascii_ok:
        return _result("BR-3")
    if not snapshot_recorded:
        return _result("BR-7")
    if fault_present:
        return _result("BR-4")
    if (activation_required and not activation_readback_ok) or not ready:
        return _result("BR-5")
    if motion_completed is None:
        return _result("BR-7")
    if not motion_completed:
        return _result("BR-6")
    return _result("BR-1")


def _result(code: str) -> dict[str, object]:
    return {
        "classification": code,
        "classification_name": _CODES[code],
        "confidence": "MEDIUM" if code == "BR-1" else "LOW",
        "confirmed": False,
        "p022_final_pass_declared": False,
        "incident": "SIMULATOR_RUNTIME_FAILURE",
        "root_cause": "NOT_CONFIRMED",
    }


def _number(value: str) -> float | None:
    if value == NOT_AVAILABLE:
        return None
    try:
        return float(value)
    except ValueError:
        return None
