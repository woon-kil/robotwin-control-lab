# P-007

PROJECT ROS 2 CONTROLLER ADAPTER

ROS 2는 transport다. Canonical command는 MOVE_JOINT다.
Controller backend는 VirtualPmacController다.
실제 로봇과 실제 Power PMAC는 사용하지 않았다.
P-008은 시작하지 않았다.

RESULT PASS
timeouts 0
non_finite 0
max_final_error_rad 0.0
max_final_error_deg 0.0
observed_feedback_hz 100.00364838337862
latency_mean_ms 1.0758933496981626
latency_p50_ms 1.0724149979068898
latency_p95_ms 1.0901190034928732
latency_p99_ms 1.1603910024859942
latency_max_ms 2.200997005274985

Adapter shutdown calls disconnect() on the controller. An active MOVE_JOINT is aborted with execution REJECTED and error_code DISCONNECTED. controller_state becomes DISCONNECTED. actual joint positions are left where they are. This is session teardown, not an E-stop and not a safety-certified stop.

NOT A ROS 2 REAL-TIME CERTIFIED CONTROLLER
NOT REAL ROBOT VALIDATED
NOT VERIFIED AGAINST REAL POWER PMAC
