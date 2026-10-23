"""Validator가 SAFE_TO_ATTEMPT일 때만 controller.submit을 호출한다."""

from __future__ import annotations

from controller.virtual_pmac import JointCommand, RobotState, VirtualPmacController
from validator.validate import validate


class ExecutionGate:
    def __init__(self, controller: VirtualPmacController) -> None:
        self.controller = controller
        self.submit_count = 0
        self.last_report: dict | None = None

    def consider(
        self,
        command: JointCommand,
        state: RobotState,
        workcell: dict,
        config: dict | None = None,
    ) -> dict:
        report = validate(command, state, workcell, config)
        self.last_report = report
        if report["decision"] != "SAFE_TO_ATTEMPT":
            report["controller_submitted"] = False
            report["controller_result"] = None
            return report
        self.submit_count += 1
        result = self.controller.submit(command)
        report["controller_submitted"] = True
        report["controller_result"] = {
            "result": result.result,
            "error_code": result.error_code,
            "trace": list(self.controller.trace(command.command_id)),
        }
        return report
