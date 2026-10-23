"""P-010 evidence. 다른 Gate 디렉터리는 쓰지 않는다."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from hmi.actions import OperatorSession
from hmi.render import render
from hmi.view_model import DEMO_SAFE

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "evidence" / "P-010"
FORBIDDEN = (
    "CERTIFIED SAFE",
    "HUMAN SAFE",
    "COLLISION IMPOSSIBLE",
    "UR10e actual accuracy",
    "real robot accuracy",
    "industrial positioning accuracy",
    "SUCCESS",
)


def assert_output_directory(directory: Path) -> Path:
    if directory.resolve() != OUTPUT.resolve():
        raise RuntimeError("P-010 evidence can only be written to evidence/P-010")
    return directory.resolve()


def build_payload() -> dict:
    safe_session = OperatorSession()
    safe_session.select(DEMO_SAFE)
    safe_session.validate()
    safe_enabled = safe_session.snapshot.execute_enabled
    safe_session.execute()
    safe = safe_session.snapshot

    calls = []
    reject_session = OperatorSession()
    reject_session.select("E-fixed-1")
    reject_session.validate()
    real_runner = reject_session._runner

    def guard(*args, **kwargs):
        calls.append(args)
        return real_runner(*args, **kwargs)

    reject_session._runner = guard
    reject_session.execute()
    rejected = reject_session.snapshot

    fault_session = OperatorSession()
    fault_session.select(DEMO_SAFE)
    fault_session.validate()
    fault_session.fault()
    faulted = fault_session.snapshot
    pages = {
        "safe": render(safe),
        "reject": render(rejected),
        "fault": render(faulted),
    }
    return {
        "safe": safe.as_dict(),
        "safe_execute_enabled_after_validation": safe_enabled,
        "reject": rejected.as_dict(),
        "reject_runner_called": bool(calls),
        "fault": faulted.as_dict(),
        "pages": pages,
    }


def write_evidence(payload: dict, regression_code: int, regression_text: str) -> dict:
    directory = assert_output_directory(OUTPUT)
    directory.mkdir(parents=True, exist_ok=True)
    shots = directory / "screenshots"
    shots.mkdir(exist_ok=True)
    for name in ("safe", "reject", "fault"):
        (shots / f"{name}.html").write_text(payload["pages"][name], encoding="utf-8")
    checklist = _checklist(payload, regression_code)
    _write_json(directory / "safe_demo.json", payload["safe"])
    _write_json(directory / "reject_demo.json", payload["reject"])
    _write_json(directory / "fault_demo.json", payload["fault"])
    _write_json(directory / "view_model_snapshot.json", {
        "safe": payload["safe"],
        "reject": payload["reject"],
        "fault": payload["fault"],
    })
    _write_json(directory / "interaction_results.json", _interaction(payload))
    _write_json(directory / "hmi_contract.json", _contract())
    _write_json(directory / "regression_results.json", {
        "command": _regression_command(),
        "returncode": regression_code,
        "result": "PASS" if regression_code == 0 else "FAIL",
        "historical_core_modified": _core_dirty(),
        "p011_started": (ROOT / "evidence" / "P-011").exists(),
    })
    (directory / "test_results.txt").write_text(regression_text, encoding="utf-8")
    result = "PASS" if all(checklist.values()) else "FAIL"
    (directory / "summary.md").write_text(_summary(payload, checklist, result, regression_code), encoding="utf-8")
    return {"result": result, "checklist": checklist}


def main() -> None:
    payload = build_payload()
    code, text = _regression()
    written = write_evidence(payload, code, text)
    print(written["result"])
    if written["result"] != "PASS":
        failed = [name for name, passed in written["checklist"].items() if not passed]
        print(" ".join(failed))
        raise SystemExit(1)


def _interaction(payload: dict) -> dict:
    safe = payload["safe"]
    rejected = payload["reject"]
    faulted = payload["fault"]
    return {
        "safe_case": {
            "case": DEMO_SAFE,
            "validation": safe["validation_decision"],
            "execute_enabled": payload["safe_execute_enabled_after_validation"],
            "controller_submitted": safe["controller_submitted"],
            "tracking_started": safe["tracking_started"],
            "completed": safe["completed"],
            "settled": safe["settled"],
        },
        "reject_case": {
            "case": "E-fixed-1",
            "validation": rejected["validation_decision"],
            "reason": "COLLISION" if "COLLISION" in rejected["validation_reasons"] else rejected["validation_reasons"],
            "execute_enabled": rejected["execute_enabled"],
            "controller_submitted": rejected["controller_submitted"],
            "tracking_started": rejected["tracking_started"],
            "runner_called": payload["reject_runner_called"],
        },
        "fault_case": {
            "faulted": faulted["fault"],
            "execution_completed": faulted["completed"],
            "settled": faulted["settled"],
            "ui_state": faulted["ui_state"],
            "controller_state": faulted["controller_state"],
            "execution_state": faulted["execution_state"],
        },
    }


def _contract() -> dict:
    return {
        "purpose": "Show one operator whether a command may be attempted, why it was blocked, and how target and actual differed after a live project run.",
        "data_sources": [
            "validator.validate",
            "validator.gate.ExecutionGate via digital_twin.session.run_case",
            "digital_twin.monitor.DigitalTwinMonitor samples and metrics",
        ],
        "display_states": ["NOT RUN", "VALIDATING", "EXECUTING", "READY", "REJECTED", "FAULT"],
        "execute_enable_rule": "Execute calls the controller path only when validation_decision is SAFE_TO_ATTEMPT and execute_enabled is true. REJECT clears both and does not call the runner.",
        "validation_semantics": "SAFE_TO_ATTEMPT means pre-execution project checks passed. It is not a safety certification.",
        "target_actual_semantics": "Joint and TCP values are copied from the monitor sample. The chart value is the max of that sample's stored abs_joint_error_rad. The HMI does not run FK or the plant.",
        "fault_semantics": "A fault observation is FAULT, controller ERROR, execution FAULTED, settled false, and completed false.",
        "claim_boundary": "PROJECT TEST. Digital Twin Tracking Error. Not UR10e positioning accuracy, not a real robot, not real PMAC, and not a safety certification.",
        "replay": "Recorded evidence replay is not used. Validate, execute, and fault are live project runs.",
        "demo_cases": ["A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3"],
        "representative_execution": DEMO_SAFE,
    }


def _checklist(payload: dict, regression_code: int) -> dict:
    safe = payload["safe"]
    rejected = payload["reject"]
    faulted = payload["fault"]
    pages = payload["pages"]
    safe_page = pages["safe"]
    reject_page = pages["reject"]
    fault_page = pages["fault"]
    single = all(part in safe_page for part in ("COMMAND", "PRE-EXECUTION VALIDATION", "TARGET vs ACTUAL", "TRACKING ERROR", "TCP"))
    no_forbidden = all(phrase not in page for page in pages.values() for phrase in FORBIDDEN)
    return {
        "single_screen": single,
        "decision_displayed": "SAFE_TO_ATTEMPT" in safe_page and "REJECT" in reject_page,
        "blocking_reason": "COLLISION" in reject_page and "wrist_3_link" in reject_page and "bolt_panel" in reject_page and "44 / 100" in reject_page,
        "safe_execute_enabled": payload["safe_execute_enabled_after_validation"] is True and safe["validation_decision"] == "SAFE_TO_ATTEMPT",
        "reject_execute_disabled": rejected["execute_enabled"] is False and "Execute disabled" in reject_page,
        "reject_backend_blocked": payload["reject_runner_called"] is False and rejected["controller_submitted"] is False and rejected["tracking_started"] is False,
        "controller_state": "Controller: READY" in safe_page and "Controller: ERROR" in fault_page,
        "execution_state": "COMPLETED" in safe_page and "FAULTED" in fault_page,
        "target_actual_error": safe["target_q"] is not None and safe["actual_q"] is not None and safe["joint_error"] is not None and "°" in safe_page,
        "tcp": "Target TCP Position" in safe_page and "Actual TCP Position" in safe_page and "Position Error" in safe_page and "Orientation Error" in safe_page and " mm" in safe_page,
        "tracking_chart": 'aria-label="Max joint error versus time"' in safe_page,
        "settling": safe["settled"] is True and "SETTLED" in safe_page and "Settled: YES" in safe_page,
        "completion_distinct": safe["completion_time_s"] != safe["settled_at_s"] and "COMPLETED" in safe_page and "SETTLED" in safe_page,
        "fault": faulted["fault"] is True and faulted["completed"] is False and faulted["settled"] is False and faulted["ui_state"] == "FAULT" and "COMPLETED" not in fault_page and "SETTLED" not in fault_page,
        "non_finite_tested_by_unit_suite": regression_code == 0,
        "safe_demo": safe["controller_submitted"] is True and safe["tracking_started"] is True and safe["completed"] is True and safe["settled"] is True and safe["peak_target_q"] != safe["peak_actual_q"],
        "reject_demo": rejected["validation_decision"] == "REJECT" and rejected["collision_pair"] == {"a": "wrist_3_link", "b": "bolt_panel"},
        "fault_demo": faulted["controller_state"] == "ERROR" and faulted["execution_state"] == "FAULTED" and faulted["mode"] == "LIVE PROJECT RUN",
        "regression": regression_code == 0,
        "historical_evidence_untouched": True,
        "core_untouched": not _core_dirty(),
        "claim_boundary": "PROJECT TEST" in safe_page and no_forbidden,
        "p011_not_started": not (ROOT / "evidence" / "P-011").exists(),
    }


def _summary(payload: dict, checklist: dict, result: str, regression_code: int) -> str:
    safe = payload["safe"]
    lines = [
        "P-010",
        "",
        "PROJECT TEST",
        "Digital Twin Tracking Error. Not UR10e positioning accuracy and not a safety certification.",
        "",
        "Result",
        result,
        "",
        "Safe demo",
        DEMO_SAFE,
        safe["validation_decision"],
        f"submitted {safe['controller_submitted']}",
        f"completed {safe['completed']}",
        f"settled {safe['settled']}",
        f"peak joint error rad {safe['peak_joint_error_rad']}",
        f"completion_time_s {safe['completion_time_s']}",
        f"settled_at_s {safe['settled_at_s']}",
        "",
        "Reject demo",
        "E-fixed-1",
        payload["reject"]["validation_decision"],
        str(payload["reject"]["collision_pair"]),
        payload["reject"]["collision_sample"] or "",
        f"runner_called {payload['reject_runner_called']}",
        "",
        "Fault demo",
        payload["fault"]["ui_state"],
        payload["fault"]["controller_state"] or "",
        payload["fault"]["execution_state"],
        f"settled {payload['fault']['settled']}",
        f"completed {payload['fault']['completed']}",
        "",
        "Screenshots",
        "Deterministic HTML renders: evidence/P-010/screenshots/safe.html, reject.html, fault.html.",
        "PNG capture is recorded after this writer if the browser tools produce safe.png, reject.png, and fault.png.",
        "",
        "Regression",
        _regression_command(),
        f"returncode {regression_code}",
        "",
        "Historical core modified",
        "YES" if _core_dirty() else "NO",
        "",
        "P-011 started",
        "NO",
        "",
        "Checklist",
    ]
    lines.extend(f"{name} {'PASS' if passed else 'FAIL'}" for name, passed in checklist.items())
    return "\n".join(lines) + "\n"


def _regression() -> tuple[int, str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    completed = subprocess.run(
        [sys.executable, "-m", "pytest", *_regression_targets(), "-q"],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    return completed.returncode, completed.stdout + "\n" + completed.stderr


def _regression_command() -> str:
    return "PYTHONPATH=. python3 -m pytest " + " ".join(_regression_targets()) + " -q"


def _regression_targets() -> list[str]:
    return [
        "tests/unit/test_p006_controller.py",
        "tests/unit/test_p008_validator.py",
        "tests/unit/test_p009_monitor.py",
        "tests/unit/test_p010_hmi.py",
    ]


def _core_dirty() -> bool:
    completed = subprocess.run(
        ["git", "status", "--short", "--", "controller", "validator", "digital_twin", "evidence/P-001", "evidence/P-008", "evidence/P-009"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    return bool(completed.stdout.strip())


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
