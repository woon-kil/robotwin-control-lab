"""P-021A read-only Motor[1] motion comparison.

이 모듈은 recovery acceptance를 바꾸지 않는다.
ACK는 motion completion이 아니다.
jog 이외의 Motor[1] configuration write를 보내지 않는다.
"""

from __future__ import annotations

import json
import platform
from datetime import datetime, timezone
from pathlib import Path

from controller.power_pmac_motor import parse_number

SNAPSHOT_VARIABLES = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].DesPos",
    "Motor[1].DesVel",
    "Motor[1].DesVelZero",
    "Motor[1].JogPos",
    "Motor[1].ProgJogPos",
    "Motor[1].JogSpeed",
    "Motor[1].JogTa",
    "Motor[1].JogTs",
    "Motor[1].MaxSpeed",
    "Motor[1].AmpEna",
    "Motor[1].AmpFault",
    "Motor[1].FeFatal",
    "Motor[1].FeWarn",
    "Motor[1].FatalFeLimit",
    "Motor[1].WarnFeLimit",
    "Motor[1].PlusLimit",
    "Motor[1].MinusLimit",
    "Motor[1].InPos",
    "Motor[1].Coord",
    "Motor[1].Ctrl",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
    "Motor[1].ActVel",
    "Motor[1].LimitStop",
)

POSITION_KEYS = (
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].DesPos",
)
RESIDUE_KEYS = POSITION_KEYS + (
    "Motor[1].JogPos",
    "Motor[1].ProgJogPos",
    "Motor[1].DesVel",
    "Motor[1].ActVel",
    "Motor[1].DesVelZero",
    "Motor[1].InPos",
)
NOT_AVAILABLE = "NOT_AVAILABLE"
CLASSIFICATIONS = {
    "RC-1": "RECONNECT_STATE_DRIFT",
    "RC-2": "RECONNECT_WITHOUT_VISIBLE_STATE_DRIFT",
    "RC-3": "BASELINE_NO_LONGER_REPRODUCIBLE",
    "RC-4": "INTERMITTENT",
    "RC-5": "ROOT_CAUSE_CONFIRMED",
    "NOT_REPRODUCED": "RECONNECT_FAILURE_NOT_REPRODUCED",
}


def parse_snapshot(stdout: str, names: tuple[str, ...] = SNAPSHOT_VARIABLES) -> dict[str, str]:
    """Assignment만 기록한다. 응답에 없는 이름은 NOT_AVAILABLE이다."""
    found: dict[str, str] = {}
    for line in stdout.splitlines():
        if "=" not in line:
            continue
        name, raw = line.split("=", 1)
        name = name.strip()
        value = raw.strip()
        if name and value:
            found[name] = value
    return {name: found.get(name, NOT_AVAILABLE) for name in names}


def values_equal(left: str, right: str) -> bool:
    if left == NOT_AVAILABLE or right == NOT_AVAILABLE:
        return False
    if left == right:
        return True
    left_number = parse_number(left)
    right_number = parse_number(right)
    if left_number is None or right_number is None:
        return False
    return left_number == right_number


def command_observation(stdout: str, stderr: str, returncode: int) -> dict[str, object]:
    combined = stdout + "\n" + stderr
    lowered = combined.lower()
    return {
        "stdout": stdout,
        "stderr": stderr,
        "returncode": returncode,
        "ack_present": "\x06" in combined,
        "error_present": "error #" in lowered,
        "ack_is_motion_success": False,
    }


def choose_small_target(current: str | None, avoid: tuple[int, ...] = ()) -> int:
    current_number = None if current in (None, NOT_AVAILABLE) else parse_number(current)
    for candidate in (1, 0, 2, 3, 4, 5):
        if candidate in avoid:
            continue
        if current_number is None or candidate != current_number:
            return candidate
    raise ValueError("no small Motor[1] target is available")


def motion_observation(before: dict[str, str], after: dict[str, str], target: int) -> dict[str, object]:
    """DesPos, Pos, ActPos가 target과 같을 때만 reached다. ACK는 보지 않는다."""
    target_number = float(target)
    before_numbers = {key: _number(before, key) for key in POSITION_KEYS}
    after_numbers = {key: _number(after, key) for key in POSITION_KEYS}
    already = all(value == target_number for value in before_numbers.values())
    reached = (not already) and all(value == target_number for value in after_numbers.values())
    return {
        "target": target,
        "units": "pmac_motor_unit",
        "reached": reached,
        "already_at_target": already,
        "despos_at_target": after_numbers["Motor[1].DesPos"] == target_number,
        "pos_at_target": after_numbers["Motor[1].Pos"] == target_number,
        "actpos_at_target": after_numbers["Motor[1].ActPos"] == target_number,
        "despos_changed": _changed(before, after, "Motor[1].DesPos"),
        "pos_changed": _changed(before, after, "Motor[1].Pos"),
        "actpos_changed": _changed(before, after, "Motor[1].ActPos"),
        "jogpos_changed": _changed(before, after, "Motor[1].JogPos"),
        "progjogpos_changed": _changed(before, after, "Motor[1].ProgJogPos"),
        "desvel_changed": _changed(before, after, "Motor[1].DesVel"),
        "ack_is_motion_success": False,
    }


def snapshot_diff(
    a_before: dict[str, str],
    a_after: dict[str, str],
    b_before: dict[str, str] | None = None,
    b_after: dict[str, str] | None = None,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for name in SNAPSHOT_VARIABLES:
        values = {
            "a_before": a_before.get(name, NOT_AVAILABLE),
            "a_after": a_after.get(name, NOT_AVAILABLE),
            "b_before": None if b_before is None else b_before.get(name, NOT_AVAILABLE),
            "b_after": None if b_after is None else b_after.get(name, NOT_AVAILABLE),
        }
        present = [values["a_before"], values["a_after"]]
        if b_before is not None:
            present.append(values["b_before"])
        if b_after is not None:
            present.append(values["b_after"])
        tags: list[str] = []
        if any(value == NOT_AVAILABLE for value in present):
            tags.append(NOT_AVAILABLE)
        else:
            motion = not values_equal(str(values["a_before"]), str(values["a_after"]))
            if b_before is not None and b_after is not None:
                motion = motion or not values_equal(str(values["b_before"]), str(values["b_after"]))
            reconnect = b_before is not None and not values_equal(str(values["a_after"]), str(values["b_before"]))
            if motion:
                tags.append("CHANGED_ON_MOTION")
            if reconnect:
                tags.append("CHANGED_AFTER_RECONNECT")
            if not tags:
                tags.append("UNCHANGED")
        rows.append({"variable": name, "tags": tags, **values})
    return rows


def explanatory_drift(fresh_before: dict[str, str], reconnect_before: dict[str, str]) -> list[str]:
    """성공한 motion이 남긴 위치 잔차 이외의 pre-jog 차이를 고른다."""
    found: list[str] = []
    for name in SNAPSHOT_VARIABLES:
        if name in RESIDUE_KEYS:
            continue
        left = fresh_before.get(name, NOT_AVAILABLE)
        right = reconnect_before.get(name, NOT_AVAILABLE)
        if left == NOT_AVAILABLE or right == NOT_AVAILABLE:
            continue
        if not values_equal(left, right):
            found.append(name)
    return found


def classify_trials(trials: list[dict[str, object]]) -> dict[str, object]:
    """상관관계만으로 RC-5를 주지 않는다."""
    if not trials:
        raise ValueError("at least one trial is required")
    fresh_raw = [trial["fresh_motion_reached"] for trial in trials]
    if any(item is None for item in fresh_raw):
        return {
            "classification": "NOT_REPRODUCED",
            "classification_name": CLASSIFICATIONS["NOT_REPRODUCED"],
            "confidence": "LOW",
            "fresh_baseline_reproducible": False,
            "reconnect_failure_reproducible": False,
            "root_cause_confirmed": False,
            "next_action": "HOLD. A trial did not finish, so no failure class is assigned.",
        }
    fresh = [bool(item) for item in fresh_raw]
    reconnect_ran = [bool(trial.get("reconnect_ran")) for trial in trials]
    reconnect = [trial.get("reconnect_motion_reached") for trial in trials if trial.get("reconnect_ran")]
    drifts = [list(trial.get("explanatory_drift", [])) for trial in trials if trial.get("reconnect_ran")]
    fresh_mixed = any(fresh) and not all(fresh)
    reconnect_mixed = any(item is True for item in reconnect) and any(item is False for item in reconnect)
    if fresh_mixed or reconnect_mixed:
        code = "RC-4"
        confidence = "MEDIUM"
        confirmed = False
        next_action = "HOLD. Same-condition PASS and FAIL both occurred. Reproduce one condition before a fix."
    elif not any(fresh):
        code = "RC-3"
        confidence = "MEDIUM" if len(fresh) >= 2 else "LOW"
        confirmed = False
        next_action = "HOLD. Fresh baseline failed, so the P-021 result is not attributed to reconnect."
    elif reconnect_ran and all(reconnect_ran) and reconnect and all(item is False for item in reconnect):
        if any(drifts):
            code = "RC-1"
            confidence = "MEDIUM" if len(reconnect) >= 2 and _same_drift(drifts) else "LOW"
            confirmed = False
            next_action = "HOLD. A state difference is visible. It is not a confirmed cause without a single-variable intervention."
        else:
            code = "RC-2"
            confidence = "MEDIUM" if len(reconnect) >= 2 else "LOW"
            confirmed = False
            next_action = "HOLD. Reconnect motion failed without an explanatory difference in the measured set."
    elif reconnect and all(item is True for item in reconnect):
        code = "NOT_REPRODUCED"
        confidence = "MEDIUM" if len(reconnect) >= 2 else "LOW"
        confirmed = False
        next_action = "HOLD. This run did not reproduce the P-021 position stall. P-021 is not closed."
    else:
        code = "RC-3" if not all(fresh) else "NOT_REPRODUCED"
        confidence = "LOW"
        confirmed = False
        next_action = "HOLD. The trial set does not isolate reconnect."
    return {
        "classification": code,
        "classification_name": CLASSIFICATIONS[code],
        "confidence": confidence,
        "fresh_baseline_reproducible": all(fresh),
        "reconnect_failure_reproducible": bool(reconnect) and all(item is False for item in reconnect),
        "root_cause_confirmed": confirmed,
        "next_action": next_action,
    }


def confirm_root_cause(condition_present_fails: bool, condition_absent_succeeds: bool) -> bool:
    """두 방향이 모두 측정된 경우에만 RC-5 조건을 만족한다."""
    return condition_present_fails and condition_absent_succeeds


def _same_drift(drifts: list[list[str]]) -> bool:
    if len(drifts) < 2:
        return False
    first = tuple(drifts[0])
    return all(tuple(item) == first for item in drifts[1:])


def _number(snapshot: dict[str, str], key: str) -> float | None:
    raw = snapshot.get(key, NOT_AVAILABLE)
    if raw == NOT_AVAILABLE:
        return None
    return parse_number(raw)


def _changed(before: dict[str, str], after: dict[str, str], key: str) -> bool:
    left = before.get(key, NOT_AVAILABLE)
    right = after.get(key, NOT_AVAILABLE)
    if left == NOT_AVAILABLE or right == NOT_AVAILABLE:
        return False
    return not values_equal(left, right)


def historical_comparison() -> dict[str, object]:
    """기존 evidence에서 옮긴 비교다. 그 파일은 다시 측정하거나 수정하지 않는다."""
    return {
        "source_note": "Values are copied from immutable evidence files. P-021A did not rewrite them.",
        "P-019B": {
            "command": "#1j=1",
            "before": {"pos": "0", "actpos": "0", "despos": "0"},
            "after": {"pos": "1", "actpos": "1", "despos": "1"},
            "ack_present": True,
            "error_present": False,
            "returncode": 1,
            "positions_changed": True,
        },
        "P-020": {
            "command": "#1j=0",
            "before": {"pos": "3", "actpos": "3", "despos": "3"},
            "after": {"pos": "0", "actpos": "0", "despos": "0"},
            "ack_present": True,
            "error_present": False,
            "result": "COMPLETED",
            "poll_count": 1,
            "elapsed_s": 6.262350667035207,
            "positions_changed": True,
        },
        "P-021": {
            "command": "#1j=2",
            "before": {"pos": "0", "actpos": "0", "despos": "0"},
            "after": {"pos": "0", "actpos": "0", "despos": "0"},
            "ack_present": True,
            "error_present": False,
            "result": "FAULTED",
            "error_code": "TIMEOUT",
            "poll_count": 4,
            "elapsed_s": 13.178414833033457,
            "positions_changed": False,
        },
    }


def write_evidence(root: Path, payload: dict[str, object]) -> None:
    root.mkdir(parents=True, exist_ok=True)
    raw = root / "raw"
    raw.mkdir(exist_ok=True)
    trials = payload["trials"]
    assert isinstance(trials, list)
    for index, trial in enumerate(trials, start=1):
        _write_json(root / f"trial_{index}.json", trial)
        rows = trial.get("rows", [])
        lines = [json.dumps(row, ensure_ascii=False) for row in rows]
        (raw / f"trial_{index}.jsonl").write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    _write_json(root / "snapshot_matrix.json", payload["snapshot_matrix"])
    _write_json(root / "snapshot_diff.json", payload["snapshot_diff"])
    _write_json(root / "historical_comparison.json", payload["historical_comparison"])
    _write_json(root / "root_cause_analysis.json", payload["root_cause_analysis"])
    (root / "environment.md").write_text(str(payload["environment_md"]), encoding="utf-8")
    (root / "summary.md").write_text(str(payload["summary_md"]), encoding="utf-8")


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def render_summary(analysis: dict[str, object], trials: list[dict[str, object]]) -> str:
    lines = [
        "# P-021A summary",
        "",
        f"측정: {datetime.now(timezone.utc).isoformat()}",
        "",
        "이 문서는 root-cause investigation이다. P-021 FINAL PASS를 선언하지 않는다. P-021은 HOLD다. P-022는 시작하지 않는다.",
        "",
        f"classification: {analysis['classification']} {analysis['classification_name']}",
        f"confidence: {analysis['confidence']}",
        f"fresh_baseline_reproducible: {analysis['fresh_baseline_reproducible']}",
        f"reconnect_failure_reproducible: {analysis['reconnect_failure_reproducible']}",
        f"root_cause_confirmed: {analysis['root_cause_confirmed']}",
        f"next_action: {analysis['next_action']}",
        "",
    ]
    for index, trial in enumerate(trials, start=1):
        fresh = trial["fresh"]
        lines.append(f"## Trial {index}")
        lines.append("")
        lines.append(f"fresh command: {fresh['command']} reached: {fresh['motion']['reached']}")
        lines.append(
            "fresh Pos/ActPos/DesPos before: "
            f"{fresh['before'].get('Motor[1].Pos')}/"
            f"{fresh['before'].get('Motor[1].ActPos')}/"
            f"{fresh['before'].get('Motor[1].DesPos')}"
        )
        lines.append(
            "fresh Pos/ActPos/DesPos after: "
            f"{fresh['after'].get('Motor[1].Pos')}/"
            f"{fresh['after'].get('Motor[1].ActPos')}/"
            f"{fresh['after'].get('Motor[1].DesPos')}"
        )
        lines.append(
            f"fresh ack: {fresh['response']['ack_present']} error: {fresh['response']['error_present']} "
            f"returncode: {fresh['response']['returncode']}"
        )
        reconnect = trial.get("reconnect")
        if not isinstance(reconnect, dict):
            lines.append("reconnect: NOT_RUN")
        else:
            lines.append(
                f"reconnect command: {reconnect['command']} reached: {reconnect['motion']['reached']} "
                f"blocked_sent: {reconnect['disconnected_command_sent']}"
            )
            lines.append(
                "reconnect Pos/ActPos/DesPos before: "
                f"{reconnect['before'].get('Motor[1].Pos')}/"
                f"{reconnect['before'].get('Motor[1].ActPos')}/"
                f"{reconnect['before'].get('Motor[1].DesPos')}"
            )
            lines.append(
                "reconnect Pos/ActPos/DesPos after: "
                f"{reconnect['after'].get('Motor[1].Pos')}/"
                f"{reconnect['after'].get('Motor[1].ActPos')}/"
                f"{reconnect['after'].get('Motor[1].DesPos')}"
            )
        lines.append("")
    lines.extend(
        [
            "## Claim boundary",
            "",
            "- Root-cause investigation only",
            "- Official Power PMAC Simulator only",
            "- software virtual Motor[1]",
            "- read-only state comparison unless explicitly recorded experimental intervention",
            "- no real Power PMAC hardware",
            "- no physical motor",
            "- no physical encoder",
            "- no physical amplifier",
            "- no physical I/O",
            "- no EtherCAT",
            "- no real robot",
            "- no servo tuning",
            "- no controller commissioning",
            "- ACK is not treated as motion completion",
            "",
        ]
    )
    return "\n".join(lines)


def render_environment() -> str:
    return "\n".join(
        [
            "# P-021A environment",
            "",
            f"측정: {datetime.now(timezone.utc).isoformat()}",
            "",
            "```text",
            "client: MacBook",
            f"client_os: {platform.platform()}",
            "pmac backend: Official Power PMAC Simulator 1.2.1",
            "firmware: 2.8.3.0 stated in the P-019B brief; this runner did not query a firmware variable",
            "path: MacBook -> 192.168.0.146:22022 -> 172.20.0.200:22 -> SSH + gpascii",
            "writes: none in the read-only trials",
            "jog stimulus: #1j=<small PMAC motor unit> only",
            "credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded",
            "```",
            "",
            "P-019, P-020, P-021 evidence는 수정하지 않는다.",
            "",
        ]
    )


def build_analysis(trials: list[dict[str, object]], intervention: dict[str, object] | None = None) -> dict[str, object]:
    classified = classify_trials(trials)
    candidates: list[str] = []
    for trial in trials:
        for name in trial.get("explanatory_drift", []):
            if name not in candidates:
                candidates.append(str(name))
    eliminated = _eliminated(trials)
    performed = intervention is not None
    confirmed = False
    if performed and isinstance(intervention, dict):
        confirmed = confirm_root_cause(
            bool(intervention.get("condition_present_fails")),
            bool(intervention.get("condition_absent_succeeds")),
        )
    if confirmed:
        classified["classification"] = "RC-5"
        classified["classification_name"] = CLASSIFICATIONS["RC-5"]
        classified["confidence"] = "HIGH"
        classified["root_cause_confirmed"] = True
        classified["next_action"] = "REVIEW. A separate P-021B fix is not started by this investigation."
    classified.update(
        {
            "candidate_variables": candidates,
            "eliminated_candidates": eliminated,
            "unexplained_differences": candidates if classified["classification"] in {"RC-1", "RC-2"} else [],
            "intervention_performed": performed,
            "intervention_result": None if intervention is None else intervention.get("result"),
            "forbidden_configuration_write": False,
        }
    )
    return classified


def _eliminated(trials: list[dict[str, object]]) -> list[str]:
    """PASS trial과 FAIL trial에서 값이 같으면 그 변수는 둘을 가르지 못한다."""
    passing: list[dict[str, str]] = []
    failing: list[dict[str, str]] = []
    for trial in trials:
        fresh = trial.get("fresh")
        reconnect = trial.get("reconnect")
        if isinstance(fresh, dict) and fresh.get("motion", {}).get("reached") is True:
            passing.append(fresh["before"])
        elif isinstance(fresh, dict):
            failing.append(fresh["before"])
        if isinstance(reconnect, dict) and reconnect.get("motion", {}).get("reached") is True:
            passing.append(reconnect["before"])
        elif isinstance(reconnect, dict):
            failing.append(reconnect["before"])
    if not passing or not failing:
        return []
    eliminated: list[str] = []
    for name in SNAPSHOT_VARIABLES:
        if name in RESIDUE_KEYS:
            continue
        pass_values = {item.get(name, NOT_AVAILABLE) for item in passing}
        fail_values = {item.get(name, NOT_AVAILABLE) for item in failing}
        if len(pass_values) == 1 and pass_values == fail_values and NOT_AVAILABLE not in pass_values:
            eliminated.append(name)
    return eliminated
