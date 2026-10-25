"""P-021B JogPos to DesPos investigation helpers.

ACK는 motion completion이 아니다.
과거 evidence에 없는 값은 NOT_MEASURED다.
"""

from __future__ import annotations

NOT_MEASURED = "NOT_MEASURED"
NOT_AVAILABLE = "NOT_AVAILABLE"

HISTORICAL_VARIABLES = (
    "ServoCtrl",
    "ClosedLoop",
    "JogPos",
    "ProgJogPos",
    "JogSpeed",
    "JogTa",
    "JogTs",
    "MaxSpeed",
    "Pos",
    "ActPos",
    "DesPos",
    "DesVel",
    "DesVelZero",
    "AmpEna",
    "AmpFault",
    "FeFatal",
    "FeWarn",
    "PlusLimit",
    "MinusLimit",
    "LimitStop",
    "InPos",
    "Coord",
    "Ctrl",
    "Status[0]",
    "Status[1]",
)

P019B_MEASURED = {
    "ServoCtrl": "1",
    "ClosedLoop": "1",
    "Pos": "0",
    "ActPos": "0",
    "DesPos": "0",
    "DesVelZero": "1",
    "AmpFault": "0",
    "FeFatal": "0",
    "FeWarn": "0",
    "LimitStop": "0",
    "InPos": "1",
    "Status[0]": "$7800",
    "Status[1]": "$0",
}
P019C_MEASURED = {
    "ServoCtrl": "1",
    "ClosedLoop": "1",
    "Pos": "2",
    "ActPos": "2",
    "DesPos": "2",
    "DesVelZero": "1",
    "AmpFault": "0",
    "FeFatal": "0",
    "FeWarn": "0",
    "LimitStop": "0",
    "InPos": "1",
    "Status[0]": "$7800",
    "Status[1]": "$0",
}
P020_MEASURED = {
    "ServoCtrl": "1",
    "ClosedLoop": "1",
    "Pos": "3",
    "ActPos": "3",
    "DesPos": "3",
    "AmpFault": "0",
    "FeFatal": "0",
    "FeWarn": "0",
    "LimitStop": "0",
    "InPos": "1",
}


def historical_matrix(columns: dict[str, dict[str, str]]) -> dict[str, dict[str, str]]:
    """주어진 측정값만 채운다. 빠진 키는 NOT_MEASURED다."""
    matrix: dict[str, dict[str, str]] = {}
    for name in HISTORICAL_VARIABLES:
        matrix[name] = {column: values.get(name, NOT_MEASURED) for column, values in columns.items()}
    return matrix


def default_historical_columns() -> dict[str, dict[str, str]]:
    return {
        "P-019B_before_successful_jog": dict(P019B_MEASURED),
        "P-019C_enable_snapshot": dict(P019C_MEASURED),
        "P-020_before_successful_jog": dict(P020_MEASURED),
    }


def values_equal(left: str, right: str) -> bool:
    if left in {NOT_MEASURED, NOT_AVAILABLE} or right in {NOT_MEASURED, NOT_AVAILABLE}:
        return False
    if left == right:
        return True
    try:
        return float(left) == float(right)
    except ValueError:
        return False


def classify_pipeline(before: dict[str, str], samples: list[dict[str, str]], target: str) -> str:
    """JogPos만 target이 되고 DesPos가 출발하지 않으면 RB-1이다."""
    if not samples:
        raise ValueError("timeline samples are required")
    jog_accepted = any(values_equal(sample.get("JogPos", NOT_AVAILABLE), target) for sample in samples)
    des_started = any(
        not values_equal(before.get("DesPos", NOT_AVAILABLE), sample.get("DesPos", NOT_AVAILABLE))
        and sample.get("DesPos", NOT_AVAILABLE) != NOT_AVAILABLE
        for sample in samples
    )
    completed = [
        all(values_equal(sample.get(key, NOT_AVAILABLE), target) for key in ("DesPos", "Pos", "ActPos"))
        for sample in samples
    ]
    if any(completed) and not all(completed):
        return "RB-5"
    if jog_accepted and not des_started and not any(completed):
        return "RB-1"
    if any(completed):
        return "PIPELINE_REACHED"
    return "RB-1"


def causal_confirmation(condition_a_fails: bool, condition_b_passes: bool, restored_fails: bool) -> dict[str, object]:
    """한 번의 성공만으로 ROOT_CAUSE_CONFIRMED를 주지 않는다."""
    if condition_a_fails and condition_b_passes and restored_fails:
        return {"classification": "RB-4", "confidence": "HIGH", "confirmed": True}
    return {"classification": None, "confidence": "LOW", "confirmed": False}


def motion_completed(sample: dict[str, str], target: str, ack_present: bool) -> bool:
    del ack_present
    return all(values_equal(sample.get(key, NOT_AVAILABLE), target) for key in ("DesPos", "Pos", "ActPos"))


def classify_pmac_write(text: str) -> str:
    if text.startswith("#1j="):
        return "motion"
    if text == "#1J/":
        return "close_loop_or_jog_stop"
    if text == "Motor[1].ServoCtrl=1":
        return "activation"
    if text.startswith("P100="):
        return "test_variable"
    markers = ("JogTa=", "JogTs=", "pEnc", "pDac", "pAmp", "EncTable", "SimConfigOk=", "FatalFeLimit=", "WarnFeLimit=")
    if any(marker in text for marker in markers):
        return "configuration"
    return "other"


def jog_time_parameter_class(historical_value: str, official_says_negative_is_valid_rate: bool) -> str:
    """음수 JogTa/JogTs를 기록된 성공값 없이 원인으로 올리지 않는다."""
    if historical_value == NOT_MEASURED:
        return "NOT_ENOUGH_EVIDENCE"
    if official_says_negative_is_valid_rate:
        return "NORMAL"
    return "SUSPICIOUS"
