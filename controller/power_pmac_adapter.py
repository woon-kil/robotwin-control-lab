"""PowerPmacControllerAdapter.

P-018 controller state represents communication/integration state only.
It does not claim verified PMAC servo or motor state.

communication_state는 adapter 안에만 있다. core controller enum은 바꾸지 않는다.
get_state().controller_state는 DISCONNECTED 또는 CONNECTED만 반환한다.
"""

from __future__ import annotations

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
    """Official Power PMAC Simulator와의 통신 경계.

    motor motion, servo enable, trajectory는 여기서 보내지 않는다.
    """

    def __init__(self, transport: PowerPmacTransport) -> None:
        self._transport = transport
        self._communication = "DISCONNECTED"

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

    def _require_healthy(self) -> None:
        if self._communication == "DISCONNECTED":
            raise PowerPmacError("DISCONNECTED", "controller is disconnected")
        if self._communication != "HEALTHY":
            raise PowerPmacError("COMMUNICATION_FAILED", "communication is not healthy")

    def _refuse_motion(self) -> None:
        if self._communication == "DISCONNECTED":
            raise PowerPmacError("DISCONNECTED", "controller is disconnected")
        raise PowerPmacError("MOTION_NOT_IN_SCOPE", "P-018 does not command PMAC servo or motor motion")
