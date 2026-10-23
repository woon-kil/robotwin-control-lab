"""Validate와 Execute는 화면 밖에서 P-008 gate와 P-009 monitor를 호출한다."""

from __future__ import annotations

from collections.abc import Callable

from controller.virtual_pmac import JointCommand, VirtualPmacController
from core.model.ur10e import load_ur10e_model
from digital_twin.session import run_case, vector
from hmi.model import HmiSnapshot
from hmi.view_model import CASES, initial_snapshot, snapshot_from_run, snapshot_from_validation
from validator.validate import load_validator_config, validate
from workcell.layout import resolve_workcell

Runner = Callable[..., dict]
Validator = Callable[[str], dict]


class OperatorSession:
    def __init__(self, runner: Runner = run_case, validate_fn: Validator | None = None) -> None:
        self._runner = runner
        self._validate_fn = validate_fn or real_validate
        self.snapshot = initial_snapshot()

    def select(self, case_name: str) -> HmiSnapshot:
        if case_name not in CASES:
            raise ValueError(case_name)
        self.snapshot = initial_snapshot()
        self.snapshot.case_name = case_name
        self.snapshot.command_id = case_name
        self.snapshot.commanded_q = list(vector(case_name))
        return self.snapshot

    def validate(self) -> HmiSnapshot:
        case_name = self.snapshot.case_name
        if case_name is None:
            raise ValueError("case is not selected")
        report = self._validate_fn(case_name)
        self.snapshot = snapshot_from_validation(case_name, report)
        return self.snapshot

    def execute(self) -> HmiSnapshot:
        if not _may_run(self.snapshot):
            self.snapshot.execute_enabled = False
            self.snapshot.backend_blocked = True
            self.snapshot.controller_submitted = False
            self.snapshot.tracking_started = False
            return self.snapshot
        result = self._runner(self.snapshot.case_name)
        self.snapshot = _with_validation_details(self.snapshot, snapshot_from_run(result))
        if self.snapshot.validation_decision != "SAFE_TO_ATTEMPT" or not self.snapshot.controller_submitted:
            self.snapshot.execute_enabled = False
            self.snapshot.backend_blocked = True
            self.snapshot.controller_submitted = False
            self.snapshot.tracking_started = False
        return self.snapshot

    def fault(self) -> HmiSnapshot:
        if not _may_run(self.snapshot):
            self.snapshot.execute_enabled = False
            self.snapshot.backend_blocked = True
            self.snapshot.controller_submitted = False
            return self.snapshot
        result = self._runner(self.snapshot.case_name, fault_after_steps=1)
        self.snapshot = _with_validation_details(self.snapshot, snapshot_from_run(result))
        self.snapshot.fault = True
        self.snapshot.settled = False
        self.snapshot.completed = False
        self.snapshot.settling_time_s = None
        self.snapshot.settled_at_s = None
        self.snapshot.completion_time_s = None
        self.snapshot.execute_enabled = False
        self.snapshot.ui_state = "FAULT"
        self.snapshot.execution_state = "FAULTED"
        if self.snapshot.controller_state is None:
            self.snapshot.controller_state = "ERROR"
        return self.snapshot


def real_validate(case_name: str) -> dict:
    model = load_ur10e_model()
    resolved = resolve_workcell()
    config = load_validator_config()
    controller = VirtualPmacController(model)
    controller.connect()
    controller.servo_on()
    before = controller.get_state()
    report = validate(JointCommand(case_name, "MOVE_JOINT", vector(case_name)), before, resolved, config)
    if controller.get_state().active_command_id is not None:
        raise RuntimeError("validate must not submit a command")
    return report


def _may_run(snapshot: HmiSnapshot) -> bool:
    return snapshot.case_name is not None and snapshot.validation_decision == "SAFE_TO_ATTEMPT" and snapshot.execute_enabled


def _with_validation_details(previous: HmiSnapshot, current: HmiSnapshot) -> HmiSnapshot:
    current.commanded_q = previous.commanded_q
    if not current.validation_summary:
        current.validation_summary = previous.validation_summary
    if not current.validation_reasons:
        current.validation_reasons = list(previous.validation_reasons)
    if current.collision_pair is None:
        current.collision_pair = previous.collision_pair
    if current.collision_sample is None:
        current.collision_sample = previous.collision_sample
    if not current.checks:
        current.checks = list(previous.checks)
    return current
