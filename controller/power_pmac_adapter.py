"""PowerPmacControllerAdapter.

P-018 controller state represents communication/integration state only.
It does not claim verified PMAC servo or motor state.

communication_state는 adapter 안에만 있다. core controller enum은 바꾸지 않는다.
get_state().controller_state는 DISCONNECTED 또는 CONNECTED만 반환한다.
"""

from __future__ import annotations

from controller.power_pmac_motor import (
    MotorState,
    SingleMotorResult,
    SingleMotorSession,
    TransportExecutor,
    assess_test_motor_readiness,
    disconnected_result,
    readiness_result,
)
from controller.power_pmac_transport import (
    PowerPmacError,
    PowerPmacTransport,
    VariableRead,
    parse_integer,
    require_read_name,
    require_write,
)
from controller.virtual_pmac import CommandResult, JointCommand, RobotState

_UNMEASURED_POSE = {
    "measured": False,
    "claim": "P-018 does not read joint or TCP state from PMAC",
}


class PowerPmacControllerAdapter:
    """P-018 communication boundary와 P-019C Motor[1] test seam.

    execute_command, servo_on, step, reset은 PMAC motor command를 보내지 않는다.
    enable_test_motor, read_test_motor_state, move_test_motor는 Motor[1] 검증 seam이다.
    reconnect_and_verify와 recover_test_motor는 software test recovery다.
    hardware fault reset, safety reset, amplifier reset이 아니다.
    6축 robot API가 아니다. ExecutionGate는 이 seam을 호출하지 않는다.
    """

    def __init__(self, transport: PowerPmacTransport, motor_executor: object | None = None) -> None:
        self._transport = transport
        self._communication = "DISCONNECTED"
        if motor_executor is not None:
            self._session: SingleMotorSession | None = SingleMotorSession(motor_executor)
        elif callable(getattr(transport, "execute_lines", None)):
            self._session = SingleMotorSession(TransportExecutor(transport))
        else:
            self._session = None

    @property
    def communication_state(self) -> str:
        return self._communication

    def connect(self) -> None:
        try:
            token = self._transport.read_variable("Sys.ServoCount")
            parse_integer(token)
        except PowerPmacError:
            self._communication = "DISCONNECTED"
            raise
        self._communication = "HEALTHY"

    def disconnect(self) -> None:
        self._communication = "DISCONNECTED"
        self._transport.close()

    def probe_health(self) -> VariableRead:
        return self.read("Sys.ServoCount")

    def read(self, name: str) -> VariableRead:
        require_read_name(name)
        self._require_healthy()
        try:
            token = self._transport.read_variable(name)
            value = parse_integer(token)
        except PowerPmacError:
            self._communication = "FAILED"
            raise
        self._communication = "HEALTHY"
        return VariableRead(name, value, token)

    def write_read_back(self, name: str, value: int) -> VariableRead:
        require_write(name, value)
        self._require_healthy()
        try:
            token = self._transport.write_and_read(name, value)
            actual = parse_integer(token)
        except PowerPmacError:
            self._communication = "FAILED"
            raise
        if actual != value:
            self._communication = "FAILED"
            raise PowerPmacError("READBACK_MISMATCH", "P100 read-back did not match the written value")
        self._communication = "HEALTHY"
        return VariableRead(name, actual, token)

    def servo_on(self) -> None:
        self._refuse_motion()

    def servo_off(self) -> None:
        self._refuse_motion()

    def execute_command(self, command: JointCommand) -> CommandResult:
        if self._communication == "DISCONNECTED":
            return CommandResult(command.command_id, "REJECTED", "DISCONNECTED")
        return CommandResult(command.command_id, "REJECTED", "MOTION_NOT_IN_SCOPE")

    def step(self, dt_s: float) -> RobotState:
        del dt_s
        self._refuse_motion()
        raise PowerPmacError("MOTION_NOT_IN_SCOPE", "P-018 does not command PMAC servo or motor motion")

    def get_state(self) -> RobotState:
        controller_state = "DISCONNECTED" if self._communication == "DISCONNECTED" else "CONNECTED"
        return RobotState(
            timestamp_s=0.0,
            robot_id="not-measured",
            controller_state=controller_state,
            joint_position=(),
            joint_velocity=(),
            target_joint_position=(),
            joint_error=(),
            tcp_pose=dict(_UNMEASURED_POSE),
            command_id=None,
            active_command_id=None,
            execution_state=None,
            error_code=None,
            fault=None,
        )

    def reset(self) -> None:
        self._refuse_motion()

    def read_test_motor_state(self) -> MotorState:
        session = self._require_session()
        return session.read_test_motor_state()

    def enable_test_motor(self) -> SingleMotorResult:
        if self._communication != "HEALTHY" or self._session is None:
            return disconnected_result()
        return self._session.enable_test_motor()

    def move_test_motor(self, target: object, motor_id: int = 1) -> SingleMotorResult:
        if self._communication != "HEALTHY" or self._session is None:
            return disconnected_result()
        return self._session.move_test_motor(target, motor_id)

    def reconnect_and_verify(self) -> SingleMotorResult:
        """connect 성공을 READY로 두지 않는다. 접속 후 Motor[1]을 다시 읽는다."""
        try:
            self.connect()
        except PowerPmacError as exc:
            return readiness_result(_recovery_error(exc.code))
        return self._verify_test_motor("reconnect_and_verify")

    def recover_test_motor(self) -> SingleMotorResult:
        """현재 Motor[1] state를 다시 읽는다. fault flag를 쓰지 않는다."""
        if self._communication != "HEALTHY" or self._session is None:
            return readiness_result("DISCONNECTED")
        return self._verify_test_motor("recover_test_motor")

    def _verify_test_motor(self, action: str) -> SingleMotorResult:
        del action
        try:
            state = self.read_test_motor_state()
        except PowerPmacError as exc:
            return readiness_result(_recovery_error(exc.code))
        return readiness_result(assess_test_motor_readiness(state), state)

    def _require_session(self) -> SingleMotorSession:
        if self._communication != "HEALTHY" or self._session is None:
            raise PowerPmacError("DISCONNECTED", "single-motor seam is disconnected")
        return self._session

    def _require_healthy(self) -> None:
        if self._communication == "DISCONNECTED":
            raise PowerPmacError("DISCONNECTED", "controller is disconnected")
        if self._communication != "HEALTHY":
            raise PowerPmacError("COMMUNICATION_FAILED", "communication is not healthy")

    def _refuse_motion(self) -> None:
        if self._communication == "DISCONNECTED":
            raise PowerPmacError("DISCONNECTED", "controller is disconnected")
        raise PowerPmacError("MOTION_NOT_IN_SCOPE", "P-018 does not command PMAC servo or motor motion")


def _recovery_error(code: str) -> str:
    if code in {"CONNECTION_UNREACHABLE", "AUTHENTICATION_FAILURE", "DISCONNECTED", "TIMEOUT"}:
        return "DISCONNECTED"
    if code == "PROTOCOL_ERROR":
        return "PROTOCOL_ERROR"
    return "RECOVERY_FAILED"
