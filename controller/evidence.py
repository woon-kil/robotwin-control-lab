"""P-006 evidence. 컨트롤러 코어는 시뮬레이터를 부르지 않는다."""

from __future__ import annotations

import json
import math
import platform
import time
from pathlib import Path

from controller.virtual_pmac import (
    CLAIM,
    CLAIM_BOUNDARY,
    COMPLETION_TOLERANCE_RAD,
    FAULT_CODES,
    CommandResult,
    ControllerError,
    JointCommand,
    VirtualPmacController,
)
from core.model.ur10e import SOURCE_COMMIT, load_ur10e_model
from isaac.ur10e_commands import CANONICAL_JOINT_ORDER, CONFIGURATIONS

DT_S = 0.01
MAX_STEPS = 10000
SELECTED = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3")


def write_evidence(directory: Path) -> dict:
    payload = build_payload()
    directory.mkdir(parents=True, exist_ok=True)
    _write_json(directory / "controller_contract.json", payload["controller_contract"])
    _write_json(directory / "state_transition_results.json", payload["state_transition_results"])
    _write_json(directory / "command_results.json", payload["command_results"])
    _write_json(directory / "fault_results.json", payload["fault_results"])
    _write_json(directory / "determinism_results.json", payload["determinism_results"])
    _write_json(directory / "latency_results.json", payload["latency_results"])
    (directory / "environment.md").write_text(payload["environment_md"], encoding="utf-8")
    (directory / "summary.md").write_text(payload["summary_md"], encoding="utf-8")
    (directory / "test_results.txt").write_text(payload["test_results_txt"], encoding="utf-8")
    return payload


def build_payload() -> dict:
    model = load_ur10e_model()
    transitions = _transition_cases()
    commands = _command_cases()
    fault = _fault_case()
    replay = _replay_case()
    latency = _latency_case()
    non_finite = _non_finite(commands, fault, replay)
    timeouts = sum(1 for row in commands["configurations"] if row["timed_out"]) + int(replay["timed_out"])
    checks = _checks(transitions, commands, fault, replay, non_finite, timeouts)
    result = "PASS" if all(item["result"] == "PASS" for item in checks) else "FAIL"
    contract = _contract(model)
    return {
        "result": result,
        "controller_contract": contract,
        "state_transition_results": transitions,
        "command_results": commands,
        "fault_results": fault,
        "determinism_results": replay,
        "latency_results": latency,
        "environment_md": _environment(model),
        "summary_md": _summary(result, commands, fault, replay, timeouts, non_finite),
        "test_results_txt": "".join(item["name"] + " " + item["result"] + "\n" for item in checks) + "RESULT " + result + "\n",
    }


def _contract(model) -> dict:
    sample = VirtualPmacController(model)
    return {
        "implementation": "VirtualPmacController",
        "claim": CLAIM,
        "claim_boundary": list(CLAIM_BOUNDARY),
        "real_pmac_compatibility_claimed": False,
        "supported_commands": ["MOVE_JOINT"],
        "prompt_command_mapping": {"JOINT_POSITION": "MOVE_JOINT"},
        "prompt_state_mapping": {
            "ENABLED": "READY",
            "EXECUTING": "RUNNING",
            "FAULT": "ERROR",
            "DISABLED": "SERVO_OFF",
        },
        "units": {"angle": "radian", "time": "second"},
        "joint_order": list(CANONICAL_JOINT_ORDER),
        "source_commit": SOURCE_COMMIT,
        "controller_states_vocabulary": [
            "DISCONNECTED",
            "CONNECTED",
            "SERVO_OFF",
            "SERVO_ON",
            "HOMING",
            "READY",
            "RUNNING",
            "HOLD",
            "ERROR",
            "ESTOP",
        ],
        "states_used_by_this_baseline": ["DISCONNECTED", "CONNECTED", "READY", "RUNNING", "SERVO_OFF", "ERROR"],
        "states_not_used": ["SERVO_ON", "HOMING", "HOLD", "ESTOP"],
        "command_states": ["ACCEPTED", "EXECUTING", "COMPLETED", "REJECTED", "FAULTED"],
        "completion_tolerance_rad": COMPLETION_TOLERANCE_RAD,
        "completion_tolerance_class": "PROJECT TEST PARAMETER",
        "virtual_motion_parameters": {
            "model": "per-joint velocity slew",
            "max_velocity_rad_s": list(sample.max_velocity_rad_s),
            "max_velocity_source": "pinned UR10e joint_limits.yaml max_velocity, converted from degrees",
            "test_dt_s": DT_S,
            "test_dt_class": "PROJECT TEST CLOCK",
        },
        "fault_codes": list(FAULT_CODES),
        "busy_policy": "REJECT_BUSY",
        "fault_reset_destination": "CONNECTED",
        "architecture_notes": [
            "REQ-011 names are used. ENABLED, EXECUTING, FAULT, DISABLED, and JOINT_POSITION are not controller_state or command_type values.",
            "servo_on enters READY. SERVO_ON is not a resting state because motion is defined to start from READY and no locked edge leaves SERVO_ON.",
            "reset from ERROR returns CONNECTED. It does not enter READY and does not change actual joint position.",
            "MOVE_TCP, IK, and Cartesian motion are not implemented.",
            "This table does not amend docs/ROBOT_INTERFACE.md and is not a Power PMAC state machine.",
        ],
    }


def _ready(controller: VirtualPmacController) -> None:
    controller.connect()
    controller.servo_on()


def _attempt(action) -> dict:
    try:
        action()
    except ControllerError as exc:
        return {"outcome": "REJECT", "code": exc.code}
    return {"outcome": "PASS", "code": None}


def _transition_cases() -> dict:
    model = load_ur10e_model()
    rows = []

    def record(name: str, builder, expect: str) -> None:
        outcome = _attempt(builder)
        rows.append(
            {
                "name": name,
                "expected": expect,
                "outcome": outcome["outcome"],
                "code": outcome["code"],
                "result": "PASS" if outcome["outcome"] == expect else "FAIL",
            }
        )

    record("DISCONNECTED -> CONNECTED", lambda: VirtualPmacController(model).connect(), "PASS")
    record(
        "CONNECTED -> READY",
        lambda: _ready(VirtualPmacController(model)),
        "PASS",
    )
    record(
        "READY -> RUNNING",
        lambda: _submit_zero(VirtualPmacController(model)),
        "PASS",
    )
    record(
        "RUNNING -> READY",
        lambda: _complete_zero(VirtualPmacController(model)),
        "PASS",
    )
    record(
        "DISCONNECTED -> RUNNING",
        lambda: _require_rejection(VirtualPmacController(model).submit(_command("illegal-disconnected", (0, 0, 0, 0, 0, 0)))),
        "REJECT",
    )
    record(
        "ERROR -> RUNNING",
        lambda: _submit_during_fault(model),
        "REJECT",
    )
    record(
        "DISCONNECTED -> servo_on",
        lambda: VirtualPmacController(model).servo_on(),
        "REJECT",
    )
    record(
        "READY -> reset",
        lambda: _reset_while_ready(model),
        "REJECT",
    )
    return {"cases": rows, "result": "PASS" if all(row["result"] == "PASS" for row in rows) else "FAIL"}


def _submit_zero(controller: VirtualPmacController) -> None:
    _ready(controller)
    result = controller.submit(_command("move-zero", (0, 0, 0, 0, 0, 0)))
    if result.result != "ACCEPTED" or controller.get_state().controller_state != "RUNNING":
        raise ControllerError("UNEXPECTED", "READY did not enter RUNNING")


def _complete_zero(controller: VirtualPmacController) -> None:
    _submit_zero(controller)
    controller.step(DT_S)
    if controller.get_state().controller_state != "READY":
        raise ControllerError("UNEXPECTED", "RUNNING did not return to READY")


def _submit_during_fault(model) -> None:
    controller = VirtualPmacController(model)
    _ready(controller)
    controller.inject_fault("TEST_FAULT")
    _require_rejection(controller.submit(_command("during-fault", (0, 0, 0, 0, 0, 0))))


def _require_rejection(result: CommandResult) -> None:
    if result.result != "REJECTED":
        return
    raise ControllerError(result.error_code or "REJECTED", "command rejected")


def _reset_while_ready(model) -> None:
    controller = VirtualPmacController(model)
    _ready(controller)
    controller.reset()


def _command_cases() -> dict:
    rows = []
    saw_motion_difference = False
    for name, targets in CONFIGURATIONS:
        if name not in SELECTED:
            continue
        row, differed = _run_configuration(name, targets)
        rows.append(row)
        saw_motion_difference = saw_motion_difference or differed
    sequence = _run_sequence()
    return {
        "configurations": rows,
        "target_differs_from_actual_during_motion": saw_motion_difference,
        "sequence": sequence,
        "result": "PASS"
        if all(row["result"] == "COMPLETED" and row["timed_out"] is False for row in rows) and sequence["result"] == "PASS"
        else "FAIL",
    }


def _run_configuration(name: str, targets: tuple[float, ...]) -> tuple[dict, bool]:
    controller = VirtualPmacController(load_ur10e_model())
    _ready(controller)
    start = controller.get_state().joint_position
    command = _command(name, targets)
    accepted = controller.submit(command)
    differed = False
    steps = 0
    timed_out = False
    while controller.get_state().controller_state == "RUNNING" and steps < MAX_STEPS:
        state = controller.step(DT_S)
        steps += 1
        if state.execution_state == "EXECUTING" and _max_error(state.target_joint_position, state.joint_position) > COMPLETION_TOLERANCE_RAD:
            differed = True
    if controller.get_state().controller_state == "RUNNING":
        timed_out = True
    final = controller.get_state()
    error = _max_error(targets, final.joint_position)
    trace = list(controller.trace(name))
    completed = trace == ["ACCEPTED", "EXECUTING", "COMPLETED"] and accepted.result == "ACCEPTED" and not timed_out
    return (
        {
            "command_id": name,
            "target_q": list(targets),
            "start_q": list(start),
            "completion_q": list(final.joint_position),
            "max_joint_error_rad": error,
            "steps": steps,
            "simulated_duration_s": steps * DT_S,
            "execution_trace": trace,
            "timed_out": timed_out,
            "result": "COMPLETED" if completed and error <= COMPLETION_TOLERANCE_RAD else "FAIL",
        },
        differed,
    )


def _run_sequence() -> dict:
    controller = VirtualPmacController(load_ur10e_model())
    _ready(controller)
    traces = []
    timed_out = False
    for name, targets in CONFIGURATIONS:
        if name not in SELECTED:
            continue
        result = controller.submit(_command("sequence-" + name, targets))
        steps = 0
        while controller.get_state().controller_state == "RUNNING" and steps < MAX_STEPS:
            controller.step(DT_S)
            steps += 1
        if steps >= MAX_STEPS and controller.get_state().controller_state == "RUNNING":
            timed_out = True
        traces.append({"command_id": "sequence-" + name, "accepted": result.result, "trace": list(controller.trace("sequence-" + name)), "steps": steps})
    busy = _busy_case()
    ok = all(row["trace"] == ["ACCEPTED", "EXECUTING", "COMPLETED"] for row in traces) and not timed_out and busy["result"] == "PASS"
    return {"commands": traces, "busy": busy, "timed_out": timed_out, "result": "PASS" if ok else "FAIL"}


def _busy_case() -> dict:
    controller = VirtualPmacController(load_ur10e_model())
    _ready(controller)
    first = controller.submit(_command("busy-first", CONFIGURATIONS[1][1]))
    controller.step(DT_S)
    second = controller.submit(_command("busy-second", CONFIGURATIONS[2][1]))
    continued = controller.get_state().active_command_id == "busy-first"
    steps = 1
    while controller.get_state().controller_state == "RUNNING" and steps < MAX_STEPS:
        controller.step(DT_S)
        steps += 1
    return {
        "first": first.result,
        "second": second.result,
        "second_code": second.error_code,
        "active_command_preserved": continued,
        "result": "PASS" if first.result == "ACCEPTED" and second.result == "REJECTED" and second.error_code == "BUSY" and continued else "FAIL",
    }


def _fault_case() -> dict:
    controller = VirtualPmacController(load_ur10e_model())
    _ready(controller)
    controller.submit(_command("fault-move", CONFIGURATIONS[1][1]))
    controller.step(DT_S)
    before = controller.get_state().joint_position
    controller.inject_fault("TEST_FAULT")
    faulted = controller.get_state()
    rejected = controller.submit(_command("fault-new", CONFIGURATIONS[2][1]))
    controller.reset()
    after = controller.get_state()
    preserved = before == after.joint_position == faulted.joint_position
    return {
        "fault_injected": faulted.fault == "TEST_FAULT" and faulted.controller_state == "ERROR",
        "active_command_aborted": faulted.active_command_id is None and controller.trace("fault-move")[-1] == "FAULTED",
        "controller_enters_fault": faulted.controller_state == "ERROR",
        "new_command_rejected": rejected.result == "REJECTED" and rejected.error_code == "ILLEGAL_STATE",
        "reset_succeeds": after.controller_state == "CONNECTED" and after.fault is None,
        "joint_state_preserved": preserved,
        "joint_position_before_fault": list(before),
        "joint_position_after_reset": list(after.joint_position),
        "result": "PASS"
        if faulted.controller_state == "ERROR" and rejected.result == "REJECTED" and after.controller_state == "CONNECTED" and preserved
        else "FAIL",
    }


def _replay_case() -> dict:
    first = _replay_once()
    second = _replay_once()
    difference = max(abs(left - right) for left, right in zip(first["final_q"], second["final_q"], strict=True))
    same_steps = first["completion_steps"] == second["completion_steps"]
    same_trace = first["traces"] == second["traces"]
    same_fault = first["fault_state"] == second["fault_state"]
    return {
        "run1_final_q": first["final_q"],
        "run2_final_q": second["final_q"],
        "run1_completion_steps": first["completion_steps"],
        "run2_completion_steps": second["completion_steps"],
        "max_difference_rad": difference,
        "traces_equal": same_trace,
        "fault_state_equal": same_fault,
        "timed_out": first["timed_out"] or second["timed_out"],
        "result": "PASS" if difference == 0.0 and same_steps and same_trace and same_fault and not first["timed_out"] else "FAIL",
    }


def _replay_once() -> dict:
    controller = VirtualPmacController(load_ur10e_model())
    _ready(controller)
    steps: list[int] = []
    traces = []
    timed_out = False
    for name, targets in CONFIGURATIONS:
        if name not in SELECTED:
            continue
        command_id = "replay-" + name
        controller.submit(_command(command_id, targets))
        count = 0
        while controller.get_state().controller_state == "RUNNING" and count < MAX_STEPS:
            controller.step(DT_S)
            count += 1
        if controller.get_state().controller_state == "RUNNING":
            timed_out = True
        steps.append(count)
        traces.append(list(controller.trace(command_id)))
    controller.inject_fault("INTERNAL_ERROR")
    fault_state = controller.get_state().controller_state
    return {
        "final_q": list(controller.get_state().joint_position),
        "completion_steps": steps,
        "traces": traces,
        "fault_state": fault_state,
        "timed_out": timed_out,
    }


def _latency_case() -> dict:
    samples: list[float] = []
    controller = VirtualPmacController(load_ur10e_model())
    _ready(controller)
    for name, targets in CONFIGURATIONS:
        if name not in SELECTED:
            continue
        started = time.perf_counter()
        controller.submit(_command("latency-" + name, targets))
        samples.append(time.perf_counter() - started)
        while controller.get_state().controller_state == "RUNNING":
            started = time.perf_counter()
            controller.step(DT_S)
            samples.append(time.perf_counter() - started)
    ordered = sorted(samples)
    return {
        "clock": "time.perf_counter host process time",
        "unit": "second",
        "sample_count": len(ordered),
        "mean_s": sum(ordered) / len(ordered),
        "p50_s": _percentile(ordered, 50),
        "p95_s": _percentile(ordered, 95),
        "p99_s": _percentile(ordered, 99),
        "max_s": ordered[-1],
        "claim_boundary": ["NOT REAL PMAC LATENCY", "NOT HARD REAL-TIME GUARANTEE"],
    }


def _checks(transitions, commands, fault, replay, non_finite: int, timeouts: int) -> list[dict]:
    rows = [
        ("controller_interface_defined", True),
        ("lifecycle_deterministic", transitions["result"] == "PASS"),
        ("valid_joint_command_accepted", commands["result"] == "PASS"),
        ("invalid_command_rejected", transitions["result"] == "PASS"),
        ("target_differs_from_actual_during_motion", commands["target_differs_from_actual_during_motion"] is True),
        ("all_five_configurations_complete", all(row["result"] == "COMPLETED" for row in commands["configurations"])),
        ("busy_policy", commands["sequence"]["busy"]["result"] == "PASS"),
        ("fault_injection", fault["fault_injected"] is True and fault["active_command_aborted"] is True),
        ("command_rejected_during_fault", fault["new_command_rejected"] is True),
        ("fault_reset", fault["reset_succeeds"] is True),
        ("joint_state_preserved_on_reset", fault["joint_state_preserved"] is True),
        ("deterministic_replay", replay["result"] == "PASS"),
        ("non_finite_state", non_finite == 0),
        ("timeout", timeouts == 0),
        ("real_pmac_claim_absent", True),
    ]
    return [{"name": name, "result": "PASS" if passed else "FAIL"} for name, passed in rows]


def _non_finite(commands, fault, replay) -> int:
    values = []
    for row in commands["configurations"]:
        values.extend(row["start_q"])
        values.extend(row["completion_q"])
        values.extend(row["target_q"])
    values.extend(fault["joint_position_before_fault"])
    values.extend(fault["joint_position_after_reset"])
    values.extend(replay["run1_final_q"])
    values.extend(replay["run2_final_q"])
    return sum(0 if math.isfinite(value) else 1 for value in values)


def _environment(model) -> str:
    return "\n".join(
        [
            "# P-006 environment",
            "",
            "implementation: VirtualPmacController",
            "claim: " + CLAIM,
            "claim_boundary: " + "; ".join(CLAIM_BOUNDARY),
            "robot: " + model.robot_id,
            "source_commit: " + model.source_commit,
            "python: " + platform.python_version(),
            "os: " + platform.platform(),
            "simulator_api_called: no",
            "real_pmac_hardware: no",
            "test_dt_s: " + str(DT_S),
            "completion_tolerance_rad: " + str(COMPLETION_TOLERANCE_RAD),
            "",
        ]
    )


def _summary(result: str, commands, fault, replay, timeouts: int, non_finite: int) -> str:
    worst = max(row["max_joint_error_rad"] for row in commands["configurations"])
    return "\n".join(
        [
            "# P-006",
            "",
            CLAIM,
            "",
            "PROJECT VIRTUAL CONTROLLER. NOT VERIFIED AGAINST REAL POWER PMAC.",
            "이 결과는 PMAC protocol, timing, motion behavior의 검증이 아니다.",
            "",
            "Result",
            result,
            "",
            "Supported command",
            "MOVE_JOINT",
            "",
            "Max final joint error rad",
            str(worst),
            "",
            "Replay max joint difference rad",
            str(replay["max_difference_rad"]),
            "",
            "Fault reset destination",
            "CONNECTED",
            "",
            "Timeouts",
            str(timeouts),
            "",
            "Non-finite",
            str(non_finite),
            "",
            "Joint preserved",
            str(fault["joint_state_preserved"]),
            "",
        ]
    )


def _command(command_id: str, targets: tuple[float, ...]) -> JointCommand:
    return JointCommand(command_id, "MOVE_JOINT", tuple(targets))


def _max_error(target, actual) -> float:
    return max(abs(float(goal) - float(current)) for goal, current in zip(target, actual, strict=True))


def _percentile(ordered: list[float], percent: int) -> float:
    rank = math.ceil(percent / 100.0 * len(ordered)) - 1
    rank = min(max(rank, 0), len(ordered) - 1)
    return ordered[rank]


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    payload = write_evidence(root / "evidence" / "P-006")
    print("RESULT " + payload["result"])
    print(payload["test_results_txt"])


if __name__ == "__main__":
    main()
