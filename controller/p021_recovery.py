"""P-021 software fault/disconnect recovery.

hardware fault reset이나 safety reset이 아니다.
reconnect 성공을 READY로 두지 않는다. state를 다시 읽고 readiness를 계산한다.
"""

from __future__ import annotations

from controller.evidence import DT_S, MAX_STEPS
from controller.p020_cross_controller import ExchangeMotor, e_fixed_boundary, release_if_safe
from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_motor import SingleMotorResult, assess_test_motor_readiness, readiness_result
from controller.power_pmac_transport import GpasciiExchange, PowerPmacError, ScriptedTransport
from controller.virtual_pmac import JointCommand, VirtualPmacController
from core.kinematics.configurations import baseline_configurations
from core.model.ur10e import load_ur10e_model

PHASES = (
    "BASELINE_READY",
    "FAILURE_INJECTED",
    "DISCONNECTED",
    "FAILURE_DETECTED",
    "EXECUTION_BLOCKED",
    "RECOVERY_ATTEMPTED",
    "RECOVERY_VERIFIED",
    "READY_AGAIN",
    "POST_RECOVERY_COMMAND",
    "POST_RECOVERY_COMPLETED",
)


def move_if_recovered(adapter: PowerPmacControllerAdapter, verification: SingleMotorResult, target: object) -> SingleMotorResult:
    """RECOVERED read-back 이전에는 move_test_motor를 호출하지 않는다."""
    if verification.result != "RECOVERED":
        return verification
    if verification.state is None or assess_test_motor_readiness(verification.state) != "RECOVERED":
        return readiness_result("RECOVERY_FAILED", verification.state)
    return adapter.move_test_motor(target)


def virtual_fault_recovery() -> dict[str, object]:
    phases: list[dict[str, object]] = []
    controller = VirtualPmacController(load_ur10e_model())
    controller.connect()
    controller.servo_on()
    _phase(phases, "BASELINE_READY", communication="CONNECTED", controller_state=controller.get_state().controller_state, fault_code=None, command_sent=False, result="READY", recovery_action=None, read_back=True)
    controller.inject_fault("TEST_FAULT")
    _phase(phases, "FAILURE_INJECTED", communication="CONNECTED", controller_state=controller.get_state().controller_state, fault_code=controller.get_state().fault, command_sent=False, result="FAULTED", recovery_action=None, read_back=True)
    blocked = controller.submit(JointCommand("p021-virtual-blocked", "MOVE_JOINT", _joints("A-zero")))
    _phase(phases, "EXECUTION_BLOCKED", communication="CONNECTED", controller_state=controller.get_state().controller_state, fault_code=controller.get_state().fault, command_sent=blocked.result == "ACCEPTED", result=blocked.result, recovery_action=None, read_back=True)
    controller.reset_fault()
    after_reset = controller.get_state()
    _phase(phases, "RECOVERY_ATTEMPTED", communication="CONNECTED", controller_state=after_reset.controller_state, fault_code=after_reset.fault, command_sent=False, result="RESET", recovery_action="reset_fault", read_back=True)
    early = controller.submit(JointCommand("p021-virtual-before-ready", "MOVE_JOINT", _joints("B-positive-1")))
    _phase(phases, "FAILURE_DETECTED", communication="CONNECTED", controller_state=controller.get_state().controller_state, fault_code=controller.get_state().fault, command_sent=early.result == "ACCEPTED", result=early.result, recovery_action=None, read_back=True)
    controller.servo_on()
    ready = controller.get_state()
    _phase(phases, "READY_AGAIN", communication="CONNECTED", controller_state=ready.controller_state, fault_code=ready.fault, command_sent=False, result=ready.controller_state, recovery_action="servo_on", read_back=True)
    command = JointCommand("p021-virtual-after", "MOVE_JOINT", _joints("B-positive-1"))
    accepted = controller.submit(command)
    _phase(phases, "POST_RECOVERY_COMMAND", communication="CONNECTED", controller_state=controller.get_state().controller_state, fault_code=controller.get_state().fault, command_sent=accepted.result == "ACCEPTED", result=accepted.result, recovery_action=None, read_back=True)
    state = controller.get_state()
    steps = 0
    while state.controller_state == "RUNNING" and state.execution_state != "COMPLETED" and steps < MAX_STEPS:
        state = controller.step(DT_S)
        steps += 1
    _phase(phases, "POST_RECOVERY_COMPLETED", communication="CONNECTED", controller_state=state.controller_state, fault_code=state.fault, command_sent=True, result=state.execution_state, recovery_action=None, read_back=True)
    return {
        "source": "VirtualPmacController existing reset_fault",
        "same_recovery_implementation": False,
        "blocked_command_id": "p021-virtual-blocked",
        "blocked_result": blocked.result,
        "blocked_sent": blocked.result == "ACCEPTED",
        "before_ready_result": early.result,
        "before_ready_sent": early.result == "ACCEPTED",
        "after_reset_state": after_reset.controller_state,
        "post_command_id": command.command_id,
        "post_result": state.execution_state,
        "post_trace": list(controller.trace(command.command_id)),
        "steps": steps,
        "phases": phases,
    }


def scripted_fault_blocks(fault_name: str) -> dict[str, object]:
    field = {"AmpFault": "Motor[1].AmpFault", "FeFatal": "Motor[1].FeFatal", "LimitStop": "Motor[1].LimitStop"}[fault_name]
    executor = ExchangeMotor({field: "1"})
    adapter, _transport = _open(executor)
    result = adapter.move_test_motor(4)
    return {
        "fault": fault_name,
        "result": result.result,
        "error_code": result.error_code,
        "command_sent": result.command_sent,
        "jog_count": _jog_count(executor),
        "completed": result.result == "COMPLETED",
    }


def scripted_fault_recovery() -> dict[str, object]:
    phases: list[dict[str, object]] = []
    executor = ExchangeMotor({"Motor[1].AmpFault": "1"})
    adapter, _transport = _open(executor)
    baseline = adapter.recover_test_motor()
    _phase(phases, "FAILURE_DETECTED", communication=adapter.communication_state, controller_state=None, fault_code=None if baseline.state is None else baseline.state.fault_code, command_sent=baseline.command_sent, result=baseline.result, recovery_action="recover_test_motor", read_back=baseline.state is not None)
    blocked = move_if_recovered(adapter, baseline, 4)
    _phase(phases, "EXECUTION_BLOCKED", communication=adapter.communication_state, controller_state=None, fault_code=None if baseline.state is None else baseline.state.fault_code, command_sent=blocked.command_sent, result=blocked.result, recovery_action=None, read_back=True)
    executor.values["Motor[1].AmpFault"] = "0"
    verified = adapter.recover_test_motor()
    _phase(phases, "RECOVERY_VERIFIED", communication=adapter.communication_state, controller_state=None, fault_code=None if verified.state is None else verified.state.fault_code, command_sent=verified.command_sent, result=verified.result, recovery_action="recover_test_motor", read_back=verified.state is not None)
    moved = move_if_recovered(adapter, verified, 2)
    _phase(phases, "POST_RECOVERY_COMPLETED", communication=adapter.communication_state, controller_state=None, fault_code=None if moved.state is None else moved.state.fault_code, command_sent=moved.command_sent, result=moved.result, recovery_action=None, read_back=moved.state is not None)
    return {
        "source": "scripted AmpFault",
        "live_fault_injection": "NOT_TESTED",
        "blocked_result": blocked.result,
        "blocked_sent": blocked.command_sent,
        "verification": verified.result,
        "post_command": moved.command,
        "post_result": moved.result,
        "jog_count": _jog_count(executor),
        "phases": phases,
    }


def scripted_fault_remains() -> dict[str, object]:
    executor = ExchangeMotor({"Motor[1].AmpFault": "1"})
    adapter, _transport = _open(executor)
    verified = adapter.recover_test_motor()
    moved = move_if_recovered(adapter, verified, 2)
    return {"result": verified.result, "post_result": moved.result, "command_sent": moved.command_sent, "jog_count": _jog_count(executor)}


def scripted_fault_cleared_but_not_ready() -> dict[str, object]:
    executor = ExchangeMotor({"Motor[1].AmpFault": "0", "Motor[1].ServoCtrl": "0", "Motor[1].ClosedLoop": "0"})
    adapter, _transport = _open(executor)
    verified = adapter.recover_test_motor()
    moved = move_if_recovered(adapter, verified, 2)
    return {
        "result": verified.result,
        "post_result": moved.result,
        "command_sent": moved.command_sent,
        "jog_count": _jog_count(executor),
        "activation_count": _count(executor, "Motor[1].ServoCtrl=1"),
        "enable_count": _count(executor, "#1J/"),
    }


def scripted_warning_does_not_block_recovery() -> dict[str, object]:
    executor = ExchangeMotor({"Motor[1].FeWarn": "1"})
    adapter, _transport = _open(executor)
    verified = adapter.recover_test_motor()
    return {"result": verified.result, "fault_code": None if verified.state is None else verified.state.fault_code, "command_sent": verified.command_sent}


def scripted_disconnect_recovery() -> dict[str, object]:
    phases: list[dict[str, object]] = []
    executor = ExchangeMotor()
    adapter, transport = _open(executor)
    baseline = adapter.recover_test_motor()
    _phase(phases, "BASELINE_READY", communication=adapter.communication_state, controller_state=None, fault_code=None, command_sent=False, result=baseline.result, recovery_action="recover_test_motor", read_back=True)
    adapter.disconnect()
    blocked = adapter.move_test_motor(4)
    _phase(phases, "DISCONNECTED", communication=adapter.communication_state, controller_state=None, fault_code=None, command_sent=blocked.command_sent, result=blocked.result, recovery_action="disconnect", read_back=False)
    transport.queue_read("1")
    verified = adapter.reconnect_and_verify()
    _phase(phases, "RECOVERY_VERIFIED", communication=adapter.communication_state, controller_state=None, fault_code=None if verified.state is None else verified.state.fault_code, command_sent=verified.command_sent, result=verified.result, recovery_action="reconnect_and_verify", read_back=verified.state is not None)
    moved = move_if_recovered(adapter, verified, 2)
    _phase(phases, "POST_RECOVERY_COMPLETED", communication=adapter.communication_state, controller_state=None, fault_code=None if moved.state is None else moved.state.fault_code, command_sent=moved.command_sent, result=moved.result, recovery_action=None, read_back=moved.state is not None)
    return {
        "source": "scripted disconnect",
        "blocked_result": blocked.result,
        "blocked_error": blocked.error_code,
        "blocked_sent": blocked.command_sent,
        "verification": verified.result,
        "post_command": moved.command,
        "post_result": moved.result,
        "jog_count": _jog_count(executor),
        "phases": phases,
    }


def scripted_reconnect_does_not_assume_ready() -> dict[str, object]:
    executor = ExchangeMotor()
    adapter, transport = _open(executor)
    before = adapter.recover_test_motor()
    cached_ready = before.result
    adapter.disconnect()
    executor.values["Motor[1].ServoCtrl"] = "0"
    executor.values["Motor[1].ClosedLoop"] = "0"
    transport.queue_read("1")
    reads_before = len(executor.calls)
    verified = adapter.reconnect_and_verify()
    moved = move_if_recovered(adapter, verified, 2)
    return {
        "cached_ready": cached_ready,
        "communication": adapter.communication_state,
        "result": verified.result,
        "read_after_reconnect": len(executor.calls) > reads_before,
        "servo_ctrl": None if verified.state is None else verified.state.normalized.get("servo_ctrl"),
        "closed_loop": None if verified.state is None else verified.state.normalized.get("closed_loop"),
        "command_sent": moved.command_sent,
        "jog_count": _jog_count(executor),
    }


def scripted_reconnect_failure() -> dict[str, object]:
    executor = ExchangeMotor()
    adapter, transport = _open(executor)
    adapter.disconnect()
    transport.queue_read(PowerPmacError("CONNECTION_UNREACHABLE", "down"))
    verified = adapter.reconnect_and_verify()
    moved = move_if_recovered(adapter, verified, 2)
    return {"result": verified.result, "command_sent": moved.command_sent, "jog_count": _jog_count(executor), "communication": adapter.communication_state}


def scripted_malformed_recovery() -> dict[str, object]:
    executor = _Broken()
    adapter, _transport = _open(executor)
    verified = adapter.recover_test_motor()
    moved = move_if_recovered(adapter, verified, 2)
    return {"result": verified.result, "command_sent": moved.command_sent, "jog_count": _jog_count(executor)}


def recovery_leaves_validator_closed() -> dict[str, object]:
    executor = ExchangeMotor()
    adapter, _transport = _open(executor)
    verified = adapter.recover_test_motor()
    boundary = e_fixed_boundary()
    calls = {"virtual": 0, "pmac": 0}

    def virtual_call() -> None:
        calls["virtual"] += 1

    def pmac_call() -> None:
        calls["pmac"] += 1

    released = release_if_safe(str(boundary["validator_result"]), virtual_call, pmac_call)
    if verified.result == "RECOVERED" and released["released"] is True:
        adapter.move_test_motor(2)
    return {
        "verification": verified.result,
        "validator_result": boundary["validator_result"],
        "virtual_backend_calls": boundary["virtual_backend_calls"],
        "pmac_backend_calls": calls["pmac"],
        "released": released["released"],
        "jog_count": _jog_count(executor),
        "boundary": boundary,
    }


def recovery_metrics(records: list[dict[str, object]]) -> dict[str, object]:
    attempted = len(records)
    recovered = sum(1 for item in records if item.get("recovered") is True)
    failed_blocked = sum(1 for item in records if item.get("failed_blocked") is True)
    completed = sum(1 for item in records if item.get("post_completed") is True)
    blocked_submissions = sum(int(item.get("blocked_submissions", 0)) for item in records)
    false_recovery = sum(int(item.get("false_recovery", 0)) for item in records)
    false_completion = sum(int(item.get("false_completion", 0)) for item in records)
    return {
        "metric_scope": "project controller execution invariant",
        "physical_safety_claim": False,
        "recovery_scenarios_attempted": attempted,
        "recovery_scenarios_recovered": recovered,
        "failed_recovery_correctly_blocked": failed_blocked,
        "post_recovery_commands_completed": completed,
        "blocked_state_command_submissions": blocked_submissions,
        "false_recovery": false_recovery,
        "false_completion": false_completion,
    }


def _open(executor: object, reads: tuple[str, ...] = ("1",)) -> tuple[PowerPmacControllerAdapter, ScriptedTransport]:
    transport = ScriptedTransport()
    for item in reads:
        transport.queue_read(item)
    adapter = PowerPmacControllerAdapter(transport, motor_executor=executor)
    adapter.connect()
    return adapter, transport


def _phase(
    phases: list[dict[str, object]],
    name: str,
    *,
    communication: str | None,
    controller_state: str | None,
    fault_code: str | None,
    command_sent: bool,
    result: str | None,
    recovery_action: str | None,
    read_back: bool,
) -> None:
    if name not in PHASES:
        raise RuntimeError("phase is outside the P-021 set")
    phases.append(
        {
            "order": len(phases),
            "phase": name,
            "communication_state": communication,
            "controller_state": controller_state,
            "fault_code": fault_code,
            "command_sent": command_sent,
            "result": result,
            "recovery_action": recovery_action,
            "read_back": read_back,
        }
    )


def _joints(test_id: str) -> tuple[float, ...]:
    for row in baseline_configurations():
        if row["test_id"] == test_id:
            vector = row["joint_vector_rad"]
            if isinstance(vector, tuple):
                return vector
    raise KeyError(test_id)


def _jog_count(executor: object) -> int:
    calls = getattr(executor, "calls", [])
    return sum(1 for call in calls if len(call) == 1 and isinstance(call[0], str) and call[0].startswith("#1j="))


def _count(executor: ExchangeMotor, line: str) -> int:
    return sum(1 for call in executor.calls if call == [line])


class _Broken:
    def __init__(self) -> None:
        self.calls: list[list[str]] = []

    def execute(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        del timeout_s
        self.calls.append(list(lines))
        return GpasciiExchange("not a pmac value", "", 1)
