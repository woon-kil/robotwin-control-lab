"""P-020 semantic comparison. 두 backend의 숫자를 같은 물리량으로 두지 않는다.

Virtual joint target은 radian이다. PMAC target은 motor unit이다.
이 모듈은 VirtualPmacController와 PowerPmacControllerAdapter의 구현을 바꾸지 않는다.
"""

from __future__ import annotations

import math
from collections.abc import Callable

from controller.evidence import DT_S, MAX_STEPS
from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_motor import (
    ENABLE_COMMAND,
    SERVO_ACTIVATION,
    SingleMotorResult,
    SingleMotorSession,
)
from controller.power_pmac_transport import GpasciiExchange, ScriptedTransport
from controller.virtual_pmac import JointCommand, VirtualPmacController
from core.kinematics.configurations import baseline_configurations
from core.model.ur10e import load_ur10e_model
from validator.gate import ExecutionGate
from workcell.layout import resolve_workcell

COMPARISON_CLASSES = ("MATCH", "DIFFERENT", "NOT_COMPARABLE", "NOT_TESTED")
_NAMES = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].DesPos",
    "Motor[1].DesVelZero",
    "Motor[1].FeFatal",
    "Motor[1].FeWarn",
    "Motor[1].AmpFault",
    "Motor[1].LimitStop",
    "Motor[1].InPos",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
)


def classify(left: object, right: object, *, comparable: bool = True) -> str:
    if not comparable:
        return "NOT_COMPARABLE"
    if left is None or right is None:
        return "NOT_TESTED"
    if left != right:
        return "DIFFERENT"
    return "MATCH"


def release_if_safe(decision: str, virtual_call: Callable[[], None], pmac_call: Callable[[], None]) -> dict[str, object]:
    if decision != "SAFE_TO_ATTEMPT":
        return {"decision": decision, "virtual_calls": 0, "pmac_calls": 0, "released": False}
    virtual_call()
    pmac_call()
    return {"decision": decision, "virtual_calls": 1, "pmac_calls": 1, "released": True}


def virtual_positive_observation() -> dict[str, object]:
    controller = VirtualPmacController(load_ur10e_model())
    controller.connect()
    connected = controller.get_state().controller_state
    controller.servo_on()
    ready = controller.get_state().controller_state
    target = _joints("A-zero")
    command = JointCommand("p020-virtual-positive", "MOVE_JOINT", target)
    submitted = controller.submit(command)
    state = controller.get_state()
    steps = 0
    while state.controller_state == "RUNNING" and state.execution_state != "COMPLETED" and steps < MAX_STEPS:
        state = controller.step(DT_S)
        steps += 1
    trace = list(controller.trace(command.command_id))
    return {
        "operation": "MOVE_JOINT",
        "configuration": "A-zero",
        "units": "radian",
        "target": list(target),
        "connected_token": connected,
        "ready_token": ready,
        "submit_result": submitted.result,
        "submit_error": submitted.error_code,
        "execution_state": state.execution_state,
        "accepted": submitted.result == "ACCEPTED",
        "completed": state.execution_state == "COMPLETED",
        "command_sent": submitted.result == "ACCEPTED",
        "motion_started": "EXECUTING" in trace,
        "target_observable": len(state.target_joint_position) == len(target),
        "actual_observable": len(state.joint_position) == len(target),
        "target_state": list(state.target_joint_position),
        "actual_state": list(state.joint_position),
        "fault": state.fault,
        "fault_free": state.fault is None and state.execution_state == "COMPLETED",
        "trace": trace,
        "steps": steps,
        "controller_state": state.controller_state,
    }


def scripted_pmac_positive() -> dict[str, object]:
    executor = ExchangeMotor()
    adapter = _connected(executor)
    result = adapter.move_test_motor(2)
    observed = pmac_move_observation(adapter.communication_state, result)
    observed["jog_count"] = _jog_count(executor)
    return observed


def pmac_move_observation(communication: str | None, result: SingleMotorResult) -> dict[str, object]:
    state = result.state
    normalized = {} if state is None else state.normalized
    raw = {} if state is None else state.raw
    desired = normalized.get("desired_pos")
    pos = normalized.get("pos")
    actual = normalized.get("actual_pos")
    return {
        "communication_state": communication,
        "ready": _pmac_ready(state),
        "result": result.result,
        "error_code": result.error_code,
        "stages": list(result.stages),
        "accepted": "ACCEPTED" in result.stages,
        "completed": result.result == "COMPLETED",
        "command_sent": result.command_sent,
        "command": result.command,
        "target_observable": desired is not None,
        "actual_observable": pos is not None and actual is not None,
        "fault_free": _fault_free(normalized, None if state is None else state.fault_code, result.result == "COMPLETED"),
        "desired_pos": desired,
        "pos": pos,
        "actual_pos": actual,
        "units": "pmac_motor_unit",
        "response_text": result.response_text,
        "poll_count": result.poll_count,
        "elapsed_s": result.elapsed_s,
        "closed_loop": normalized.get("closed_loop"),
        "in_position": normalized.get("in_position"),
        "fault_code": None if state is None else state.fault_code,
        "raw_pos": raw.get("Motor[1].Pos"),
        "raw_act": raw.get("Motor[1].ActPos"),
        "raw_des": raw.get("Motor[1].DesPos"),
        "fe_fatal": normalized.get("following_error_fatal"),
        "fe_warn": normalized.get("following_error_warning"),
        "amp_fault": normalized.get("amp_fault"),
        "limit_stop": normalized.get("limit_stop"),
        "servo_ctrl": normalized.get("servo_ctrl"),
        "activation_write": result.activation_write,
        "enable_command_sent": result.enable_command_sent,
    }


def positive_rows(virtual: dict[str, object], pmac: dict[str, object]) -> list[dict[str, object]]:
    rows = [
        _judged(
            "positive",
            "connection",
            "OBSERVABLE" if virtual.get("connected_token") else None,
            {"token": virtual.get("connected_token")},
            None,
            "OBSERVABLE" if pmac.get("communication_state") else None,
            {"token": pmac.get("communication_state"), "note": "communication state, not a servo claim"},
            None,
            "Connection is observable on both sides. The tokens are not the same word.",
        ),
        _judged(
            "positive",
            "ready",
            "READY" if virtual.get("ready_token") == "READY" else "NOT_READY",
            {"token": virtual.get("ready_token")},
            None,
            "READY" if pmac.get("ready") is True else "NOT_READY",
            {
                "servo_ctrl": pmac.get("servo_ctrl"),
                "closed_loop": pmac.get("closed_loop"),
                "note": "READY here means ServoCtrl>0 and ClosedLoop==1. It is not a PMAC controller_state token.",
            },
            None,
            "Ready is recorded only from the virtual controller state and the named PMAC variables.",
        ),
        _judged(
            "positive",
            "command_acceptance",
            "ACCEPTED" if virtual.get("accepted") is True else virtual.get("submit_result"),
            {"trace": virtual.get("trace")},
            virtual.get("command_sent"),
            "ACCEPTED" if pmac.get("accepted") is True else pmac.get("result"),
            {"stages": pmac.get("stages"), "command": pmac.get("command")},
            pmac.get("command_sent"),
            "Acceptance is the semantic. Final completion is a separate row.",
            compare_sent=True,
        ),
        _judged(
            "positive",
            "motion_started",
            "OBSERVABLE" if virtual.get("motion_started") is True else "NOT_OBSERVABLE",
            {"trace": virtual.get("trace")},
            virtual.get("command_sent"),
            "OBSERVABLE" if pmac.get("command_sent") is True else "NOT_OBSERVABLE",
            {"command": pmac.get("command")},
            pmac.get("command_sent"),
            "Virtual motion start is the EXECUTING trace. PMAC motion start at this seam is command_sent.",
        ),
        _judged(
            "positive",
            "completion",
            "COMPLETED" if virtual.get("completed") is True else virtual.get("execution_state"),
            {"steps": virtual.get("steps"), "units": virtual.get("units")},
            virtual.get("command_sent"),
            "COMPLETED" if pmac.get("completed") is True else pmac.get("result"),
            {"poll_count": pmac.get("poll_count"), "elapsed_s": pmac.get("elapsed_s"), "units": pmac.get("units")},
            pmac.get("command_sent"),
            "Completion is state-based on each backend. Elapsed time is not servo latency.",
        ),
        _judged(
            "positive",
            "target_observable",
            "OBSERVABLE" if virtual.get("target_observable") is True else "NOT_OBSERVABLE",
            {"units": "radian", "target_joint_position": virtual.get("target_state")},
            None,
            "OBSERVABLE" if pmac.get("target_observable") is True else "NOT_OBSERVABLE",
            {"units": "pmac_motor_unit", "desired_pos": pmac.get("desired_pos"), "raw": pmac.get("raw_des")},
            None,
            "Both expose a target. The numbers are not compared.",
        ),
        _judged(
            "positive",
            "actual_observable",
            "OBSERVABLE" if virtual.get("actual_observable") is True else "NOT_OBSERVABLE",
            {"units": "radian", "joint_position": virtual.get("actual_state")},
            None,
            "OBSERVABLE" if pmac.get("actual_observable") is True else "NOT_OBSERVABLE",
            {"units": "pmac_motor_unit", "pos": pmac.get("pos"), "actual_pos": pmac.get("actual_pos")},
            None,
            "Both expose an actual. ActPos is simulator virtual feedback when the live backend is used.",
        ),
        _judged(
            "positive",
            "fault_free",
            "FAULT_FREE" if virtual.get("fault_free") is True else "NOT_FAULT_FREE",
            {"fault": virtual.get("fault")},
            None,
            "FAULT_FREE" if pmac.get("fault_free") is True else "NOT_FAULT_FREE",
            {
                "fault_code": pmac.get("fault_code"),
                "fe_fatal": pmac.get("fe_fatal"),
                "fe_warn": pmac.get("fe_warn"),
                "amp_fault": pmac.get("amp_fault"),
                "limit_stop": pmac.get("limit_stop"),
            },
            None,
            "FeWarn is recorded and is not promoted to a fault.",
        ),
    ]
    rows.append(numeric_row(virtual, pmac))
    return rows


def invalid_rows() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    samples = (
        ("nan", math.nan, math.nan, "NaN"),
        ("positive_infinity", math.inf, math.inf, "+Inf"),
        ("negative_infinity", -math.inf, -math.inf, "-Inf"),
        ("non_numeric", "bad", "1\n#2j=100", "non-numeric"),
    )
    for name, virtual_value, pmac_value, label in samples:
        virtual = _virtual_reject(virtual_value)
        executor = ExchangeMotor()
        adapter = _connected(executor)
        result = adapter.move_test_motor(pmac_value)
        rows.extend(
            _reject_pair(
                "invalid_" + name,
                virtual,
                result,
                _jog_count(executor),
                label,
                "Invalid input is rejected before motion on both backends. Error tokens are not required to match.",
            )
        )
    return rows


def unsupported_rows() -> list[dict[str, object]]:
    controller = _ready()
    virtual = controller.submit(JointCommand("p020-tcp", "MOVE_TCP", _joints("A-zero")))
    virtual_obs = _virtual_result(virtual, controller.get_state().fault, controller.get_state().controller_state)
    executor = ExchangeMotor()
    adapter = _connected(executor)
    result = adapter.move_test_motor(1, motor_id=2)
    return _reject_pair(
        "unsupported",
        virtual_obs,
        result,
        _jog_count(executor),
        "MOVE_TCP / motor 2",
        "Unsupported scope is rejected. Virtual uses MOVE_TCP. PMAC uses motor_id 2. Error tokens may differ.",
    )


def disconnect_rows() -> list[dict[str, object]]:
    controller = _ready()
    controller.disconnect()
    virtual = controller.submit(JointCommand("p020-disconnected", "MOVE_JOINT", _joints("A-zero")))
    state = controller.get_state()
    virtual_obs = _virtual_result(virtual, state.fault, state.controller_state)
    executor = ExchangeMotor()
    adapter = PowerPmacControllerAdapter(ScriptedTransport(), motor_executor=executor)
    result = adapter.move_test_motor(1)
    rows = _reject_pair(
        "disconnect",
        virtual_obs,
        result,
        _jog_count(executor),
        "disconnected",
        "Disconnect blocks execution. The simulator process is not stopped. Error tokens may differ.",
    )
    rows.append(
        _judged(
            "disconnect",
            "disconnected_state",
            "DISCONNECTED" if state.controller_state == "DISCONNECTED" else state.controller_state,
            {"controller_state": state.controller_state},
            False,
            "DISCONNECTED" if result.error_code == "DISCONNECTED" else result.error_code,
            {"communication_required": "HEALTHY"},
            result.command_sent,
            "Both sides report a disconnected condition. Live simulator shutdown is not required.",
            compare_sent=True,
        )
    )
    return rows


def fault_rows() -> list[dict[str, object]]:
    controller = _ready()
    controller.inject_fault("TEST_FAULT")
    virtual = controller.submit(JointCommand("p020-faulted", "MOVE_JOINT", _joints("A-zero")))
    state = controller.get_state()
    executor = ExchangeMotor({"Motor[1].AmpFault": "1"})
    adapter = _connected(executor)
    result = adapter.move_test_motor(2)
    virtual_detected = "DETECTED" if state.fault else "NOT_DETECTED"
    pmac_detected = "DETECTED" if result.result == "FAULTED" or (result.state is not None and result.state.fault_code) else "NOT_DETECTED"
    rows = [
        _judged(
            "fault",
            "fault_detected",
            virtual_detected,
            {"fault": state.fault, "controller_state": state.controller_state, "error_code": virtual.error_code},
            virtual.result == "ACCEPTED",
            pmac_detected,
            {"result": result.result, "error_code": result.error_code, "fault_code": None if result.state is None else result.state.fault_code},
            result.command_sent,
            "Fault detection does not require the same error token.",
        ),
        _judged(
            "fault",
            "command_not_executed",
            "NOT_SENT" if virtual.result != "ACCEPTED" else "SENT",
            {"result": virtual.result},
            virtual.result == "ACCEPTED",
            "NOT_SENT" if result.command_sent is False else "SENT",
            {"jog_count": _jog_count(executor)},
            result.command_sent,
            "A fault before motion blocks the command on both backends.",
            compare_sent=True,
        ),
        _judged(
            "fault",
            "not_completed",
            "NOT_COMPLETED" if controller.get_state().execution_state != "COMPLETED" else "COMPLETED",
            {"execution_state": controller.get_state().execution_state},
            virtual.result == "ACCEPTED",
            "NOT_COMPLETED" if result.result != "COMPLETED" else "COMPLETED",
            {"result": result.result},
            result.command_sent,
            "Fault must not be reported as completion.",
        ),
        _judged(
            "fault",
            "result_token",
            virtual.result,
            {"error_code": virtual.error_code},
            virtual.result == "ACCEPTED",
            result.result,
            {"error_code": result.error_code},
            result.command_sent,
            "REJECTED and FAULTED are different tokens. The block semantics are compared on the other fault rows.",
        ),
    ]
    return rows


def timeout_rows() -> list[dict[str, object]]:
    executor = ExchangeMotor(apply_target=False)
    session = SingleMotorSession(executor, now=lambda: 0.0, sleep=lambda _delay: None, timeout_s=0.0)
    result = session.move_test_motor(2)
    return [
        _judged(
            "timeout",
            "timeout",
            "NOT_PRESENT",
            {"reason": "VirtualPmacController has no command-layer timeout"},
            None,
            result.error_code,
            {
                "result": result.result,
                "completed": result.result == "COMPLETED",
                "command_sent": result.command_sent,
                "jog_count": _jog_count(executor),
                "stages": list(result.stages),
            },
            result.command_sent,
            "VirtualPmacController steps a deterministic plant until completion. A timeout was not added. The PMAC seam timed out and did not complete.",
            comparable=False,
        )
    ]


def reset_rows() -> list[dict[str, object]]:
    controller = _ready()
    controller.inject_fault("TEST_FAULT")
    held = controller.get_state().joint_position
    controller.reset_fault()
    state = controller.get_state()
    return [
        _judged(
            "reset",
            "recovery",
            "PRESENT" if state.controller_state == "CONNECTED" and state.joint_position == held else "UNEXPECTED",
            {"controller_state": state.controller_state, "joints_preserved": list(state.joint_position) == list(held)},
            False,
            "NOT_PRESENT",
            {"called": False},
            None,
            "VirtualPmacController.reset_fault exists. The PMAC seam has no fault-clear command, and none was invented.",
            comparable=False,
        )
    ]


def numeric_row(virtual: dict[str, object], pmac: dict[str, object]) -> dict[str, object]:
    return _judged(
        "positive",
        "numeric_position_equality",
        "NOT_EVALUATED",
        {"units": virtual.get("units"), "target": virtual.get("target")},
        None,
        "NOT_EVALUATED",
        {"units": pmac.get("units"), "desired_pos": pmac.get("desired_pos"), "pos": pmac.get("pos")},
        None,
        "Radian and PMAC motor unit are not compared. Equal-looking numbers are still not equivalence.",
        comparable=False,
    )


def e_fixed_boundary() -> dict[str, object]:
    controller = _ready()
    gate = ExecutionGate(controller)
    command = JointCommand("E-fixed-1", "MOVE_JOINT", _joints("E-fixed-1"))
    report = gate.consider(command, controller.get_state(), resolve_workcell())
    calls = {"virtual": 0, "pmac": 0}

    def virtual_call() -> None:
        calls["virtual"] += 1

    def pmac_call() -> None:
        calls["pmac"] += 1

    released = release_if_safe(str(report["decision"]), virtual_call, pmac_call)
    checks = report.get("checks", [])
    collision = next((item for item in checks if isinstance(item, dict) and item.get("name") == "robot_environment_collision"), None)
    metrics = collision.get("metrics") if isinstance(collision, dict) else None
    return {
        "scenario": "E-fixed-1",
        "validator_result": report.get("decision"),
        "blocking_reason_codes": list(report.get("blocking_reason_codes", [])),
        "collision_result": None if not isinstance(collision, dict) else collision.get("result"),
        "involved_objects": None if not isinstance(metrics, dict) else metrics.get("involved_objects"),
        "execution_gate_submit_count": gate.submit_count,
        "virtual_backend_calls": gate.submit_count,
        "harness_virtual_calls": calls["virtual"],
        "pmac_backend_calls": calls["pmac"],
        "released": released["released"],
        "acceptance_criterion_changed": False,
    }


def validator_rows(boundary: dict[str, object]) -> list[dict[str, object]]:
    blocked = boundary.get("validator_result") == "REJECT" and boundary.get("virtual_backend_calls") == 0 and boundary.get("pmac_backend_calls") == 0
    return [
        _judged(
            "validator_boundary",
            "controller_not_called",
            "NOT_CALLED" if boundary.get("virtual_backend_calls") == 0 else "CALLED",
            {
                "decision": boundary.get("validator_result"),
                "submit_count": boundary.get("execution_gate_submit_count"),
                "blocking_reason_codes": boundary.get("blocking_reason_codes"),
            },
            False,
            "NOT_CALLED" if boundary.get("pmac_backend_calls") == 0 else "CALLED",
            {"pmac_backend_calls": boundary.get("pmac_backend_calls"), "released": boundary.get("released")},
            False,
            "E-fixed-1 stays on the existing validator path. REJECT does not release either backend."
            if blocked
            else "Validator boundary did not keep both backends uncalled.",
            compare_sent=True,
        )
    ]


def build_matrix(pmac_positive: dict[str, object], boundary: dict[str, object] | None = None) -> tuple[dict[str, object], list[dict[str, object]]]:
    virtual = virtual_positive_observation()
    rows = positive_rows(virtual, pmac_positive)
    rows.extend(invalid_rows())
    rows.extend(unsupported_rows())
    rows.extend(disconnect_rows())
    rows.extend(fault_rows())
    rows.extend(timeout_rows())
    rows.extend(reset_rows())
    if boundary is None:
        boundary = e_fixed_boundary()
    rows.extend(validator_rows(boundary))
    return virtual, rows


def negative_matrix(rows: list[dict[str, object]]) -> dict[str, object]:
    wanted = ("invalid_nan", "invalid_positive_infinity", "invalid_negative_infinity", "invalid_non_numeric", "unsupported", "disconnect", "fault", "timeout")
    semantics = {
        "command_rejected",
        "error_code",
        "fault_not_created",
        "execution_blocked",
        "command_not_executed",
        "not_completed",
        "result_token",
        "timeout",
        "disconnected_state",
        "fault_detected",
    }
    picked = [item for item in rows if item.get("scenario") in wanted and item.get("semantic") in semantics]
    return {"cases": picked}


def state_mapping(virtual: dict[str, object], pmac: dict[str, object], rows: list[dict[str, object]]) -> dict[str, object]:
    fault = next(item for item in rows if item.get("scenario") == "fault" and item.get("semantic") == "fault_detected")
    disconnected = next(item for item in rows if item.get("scenario") == "disconnect" and item.get("semantic") == "disconnected_state")
    return {
        "values_compared": False,
        "rows": [
            {
                "semantic": "CONNECTED",
                "virtual": virtual.get("connected_token"),
                "power_pmac": pmac.get("communication_state"),
                "same_token": virtual.get("connected_token") == pmac.get("communication_state"),
                "comparison": "MATCH" if virtual.get("connected_token") and pmac.get("communication_state") else "NOT_TESTED",
                "note": "Both expose connection. HEALTHY is communication state, not servo state.",
            },
            {
                "semantic": "READY",
                "virtual": virtual.get("ready_token"),
                "power_pmac": {"servo_ctrl": pmac.get("servo_ctrl"), "closed_loop": pmac.get("closed_loop")},
                "same_token": False,
                "comparison": "MATCH" if virtual.get("ready_token") == "READY" and pmac.get("ready") is True else "NOT_TESTED",
                "note": "PMAC ready is ServoCtrl>0 and ClosedLoop==1 from named variables.",
            },
            {
                "semantic": "TARGET",
                "virtual": "target_joint_position",
                "power_pmac": "DesPos",
                "values_compared": False,
                "comparison": "NOT_COMPARABLE",
                "note": "Field correspondence only. Radian and motor unit are not equal.",
            },
            {
                "semantic": "ACTUAL",
                "virtual": "joint_position",
                "power_pmac": "Pos / ActPos",
                "values_compared": False,
                "comparison": "NOT_COMPARABLE",
                "note": "ActPos is simulator virtual feedback on the live path, not a physical encoder.",
            },
            {
                "semantic": "COMPLETED",
                "virtual": virtual.get("execution_state"),
                "power_pmac": pmac.get("result"),
                "comparison": classify(
                    "COMPLETED" if virtual.get("completed") is True else virtual.get("execution_state"),
                    "COMPLETED" if pmac.get("completed") is True else pmac.get("result"),
                ),
            },
            {
                "semantic": "FAULT",
                "virtual": fault["virtual"]["state"],
                "power_pmac": fault["power_pmac"]["state"],
                "comparison": fault["comparison"],
                "note": "Named PMAC fault variables only. Status bits are not decoded.",
            },
            {
                "semantic": "DISCONNECTED",
                "virtual": disconnected["virtual"]["result"],
                "power_pmac": disconnected["power_pmac"]["result"],
                "comparison": disconnected["comparison"],
            },
            {
                "semantic": "RESET",
                "virtual": "reset_fault",
                "power_pmac": "NOT_PRESENT",
                "comparison": "NOT_COMPARABLE",
                "note": "No PMAC fault-clear command was added.",
            },
        ],
    }


class ExchangeMotor:
    """Scripted Motor[1] exchange. Official Simulator가 아니다."""

    def __init__(self, overrides: dict[str, str] | None = None, *, apply_target: bool = True) -> None:
        self.values = {name: "0" for name in _NAMES}
        self.values["Motor[1].ServoCtrl"] = "1"
        self.values["Motor[1].ClosedLoop"] = "1"
        self.values["Motor[1].DesVelZero"] = "1"
        self.values["Motor[1].Status[0]"] = "$4000"
        if overrides:
            self.values.update(overrides)
        self.apply_target = apply_target
        self.calls: list[list[str]] = []

    def execute(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        del timeout_s
        self.calls.append(list(lines))
        if lines == [SERVO_ACTIVATION]:
            self.values["Motor[1].ServoCtrl"] = "1"
            return _exchange("")
        if lines == [ENABLE_COMMAND]:
            self.values["Motor[1].ClosedLoop"] = "1"
            return _exchange("")
        if len(lines) == 1 and lines[0].startswith("#1j="):
            if self.apply_target:
                target = lines[0].split("=", 1)[1]
                self.values["Motor[1].Pos"] = target
                self.values["Motor[1].ActPos"] = target
                self.values["Motor[1].DesPos"] = target
            return _exchange("")
        return _exchange("\n".join(name + "=" + self.values[name] for name in lines))


def _judged(
    scenario: str,
    semantic: str,
    virtual_result: object,
    virtual_state: dict[str, object],
    virtual_sent: object,
    pmac_result: object,
    pmac_state: dict[str, object],
    pmac_sent: object,
    reason: str,
    *,
    comparable: bool = True,
    compare_sent: bool = False,
) -> dict[str, object]:
    comparison = classify(virtual_result, pmac_result, comparable=comparable)
    if comparison == "MATCH" and compare_sent and virtual_sent != pmac_sent:
        comparison = "DIFFERENT"
    if comparison not in COMPARISON_CLASSES:
        raise RuntimeError("comparison class is outside the P-020 set")
    return {
        "scenario": scenario,
        "semantic": semantic,
        "virtual": {"result": virtual_result, "state": virtual_state, "command_sent": virtual_sent},
        "power_pmac": {"result": pmac_result, "state": pmac_state, "command_sent": pmac_sent},
        "comparison": comparison,
        "reason": reason,
    }


def _reject_pair(
    scenario: str,
    virtual: dict[str, object],
    result: SingleMotorResult,
    jog_count: int,
    label: str,
    reason: str,
) -> list[dict[str, object]]:
    virtual_blocked = "REJECTED" if virtual["result"] == "REJECTED" and virtual["command_sent"] is False else virtual["result"]
    pmac_blocked = "REJECTED" if result.result == "REJECTED" and result.command_sent is False else result.result
    return [
        _judged(
            scenario,
            "command_rejected",
            virtual_blocked,
            {"error_code": virtual["error_code"], "fault": virtual["fault"], "input": label},
            virtual["command_sent"],
            pmac_blocked,
            {"error_code": result.error_code, "jog_count": jog_count, "input": label},
            result.command_sent,
            reason,
            compare_sent=True,
        ),
        _judged(
            scenario,
            "error_code",
            virtual["error_code"],
            {"result": virtual["result"]},
            virtual["command_sent"],
            result.error_code,
            {"result": result.result},
            result.command_sent,
            "Error tokens stay different when the contracts use different names.",
        ),
        _judged(
            scenario,
            "fault_not_created",
            "NO_FAULT" if virtual["fault"] is None else "FAULT",
            {"fault": virtual["fault"]},
            virtual["command_sent"],
            "NO_FAULT" if result.result != "FAULTED" else "FAULT",
            {"result": result.result},
            result.command_sent,
            "Rejection must not invent a fault.",
        ),
    ]


def _virtual_reject(value: object) -> dict[str, object]:
    controller = _ready()
    result = controller.submit(JointCommand("p020-invalid", "MOVE_JOINT", (0.0, 0.0, 0.0, 0.0, 0.0, value)))
    state = controller.get_state()
    return _virtual_result(result, state.fault, state.controller_state)


def _virtual_result(result: object, fault: str | None, controller_state: str) -> dict[str, object]:
    outcome = getattr(result, "result")
    return {
        "result": outcome,
        "error_code": getattr(result, "error_code"),
        "command_sent": outcome == "ACCEPTED",
        "fault": fault,
        "controller_state": controller_state,
    }


def _ready() -> VirtualPmacController:
    controller = VirtualPmacController(load_ur10e_model())
    controller.connect()
    controller.servo_on()
    return controller


def _connected(executor: ExchangeMotor) -> PowerPmacControllerAdapter:
    transport = ScriptedTransport()
    transport.queue_read("1")
    adapter = PowerPmacControllerAdapter(transport, motor_executor=executor)
    adapter.connect()
    return adapter


def _joints(test_id: str) -> tuple[float, ...]:
    for row in baseline_configurations():
        if row["test_id"] == test_id:
            vector = row["joint_vector_rad"]
            if isinstance(vector, tuple):
                return vector
    raise KeyError(test_id)


def _pmac_ready(state: object) -> bool | None:
    normalized = getattr(state, "normalized", None)
    if not isinstance(normalized, dict):
        return None
    servo = normalized.get("servo_ctrl")
    if isinstance(servo, bool) or not isinstance(servo, (int, float)):
        return False
    return servo > 0 and normalized.get("closed_loop") is True


def _fault_free(normalized: dict[str, object], fault_code: str | None, completed: bool) -> bool:
    if not completed or fault_code is not None:
        return False
    return all(normalized.get(name) is False for name in ("following_error_fatal", "amp_fault", "limit_stop"))


def _jog_count(executor: ExchangeMotor) -> int:
    return sum(1 for call in executor.calls if len(call) == 1 and call[0].startswith("#1j="))


def _exchange(stdout: str) -> GpasciiExchange:
    return GpasciiExchange("STDIN Open for ASCII Input\n" + stdout + "\n", "", 1)
