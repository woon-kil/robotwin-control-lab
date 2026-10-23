P-010

PROJECT TEST
Digital Twin Tracking Error. Not UR10e positioning accuracy and not a safety certification.

Result
PASS

Safe demo
B-positive-1
SAFE_TO_ATTEMPT
submitted True
completed True
settled True
peak joint error rad 0.7
completion_time_s 0.23000000000000007
settled_at_s 0.25000000000000006

Reject demo
E-fixed-1
REJECT
{'a': 'wrist_3_link', 'b': 'bolt_panel'}
44 / 100
runner_called False

Fault demo
FAULT
ERROR
FAULTED
settled False
completed False

Screenshots
3
evidence/P-010/screenshots/safe.png
evidence/P-010/screenshots/reject.png
evidence/P-010/screenshots/fault.png
HTML renders are beside the PNG files.

Regression
PYTHONPATH=. python3 -m pytest tests/unit/test_p006_controller.py tests/unit/test_p008_validator.py tests/unit/test_p009_monitor.py tests/unit/test_p010_hmi.py -q
returncode 0

Historical core modified
NO

P-011 started
NO

Checklist
single_screen PASS
decision_displayed PASS
blocking_reason PASS
safe_execute_enabled PASS
reject_execute_disabled PASS
reject_backend_blocked PASS
controller_state PASS
execution_state PASS
target_actual_error PASS
tcp PASS
tracking_chart PASS
settling PASS
completion_distinct PASS
fault PASS
non_finite_tested_by_unit_suite PASS
safe_demo PASS
reject_demo PASS
fault_demo PASS
regression PASS
historical_evidence_untouched PASS
core_untouched PASS
claim_boundary PASS
p011_not_started PASS
