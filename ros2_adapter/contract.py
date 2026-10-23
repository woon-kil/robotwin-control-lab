"""P-007 transport names.

이 값들은 Canonical Robot Command가 아니다.
ROS 2 메시지가 controller core의 명령 모델이 되지 않게 이름을 여기 둔다.
"""

CANONICAL_JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

TOPIC_COMMAND = "/robotwin/command/joint"
TOPIC_RESULT = "/robotwin/command/result"
TOPIC_STATE = "/robotwin/state"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_TARGET_JOINT_STATES = "/robotwin/target_joint_states"
TOPIC_INJECT_FAULT = "/robotwin/test/inject_fault"
TOPIC_RESET_FAULT = "/robotwin/test/reset_fault"
TOPIC_SERVO_ON = "/robotwin/test/servo_on"
TOPIC_COUNT_WINDOW = "/robotwin/test/count_window"
TOPIC_SHUTDOWN = "/robotwin/test/shutdown"

# rmw_qos_profile_default. 조회 2026-10-23.
# https://raw.githubusercontent.com/ros2/rmw/jazzy/rmw/include/rmw/qos_profiles.h
# sensor_data profile(BEST_EFFORT, depth 5)은 표본을 버릴 수 있어 이번 측정에 쓰지 않는다.
# 이 설정은 산업 안전 통신 보증이 아니다.
QOS_RELIABILITY = "RELIABLE"
QOS_HISTORY = "KEEP_LAST"
QOS_DEPTH = 10
QOS_DURABILITY = "VOLATILE"
QOS_SOURCE = "https://raw.githubusercontent.com/ros2/rmw/jazzy/rmw/include/rmw/qos_profiles.h"
QOS_PROFILE_NAME = "rmw_qos_profile_default"

# 측정 전에 고정한다. 실행 결과를 보고 이 값을 맞추지 않는다.
CONFIGURED_FEEDBACK_HZ = 100.0
DT_S = 0.01
RATE_WINDOW_S = 2.0
JOINT_STATE_DRAIN_S = 1.0
LATENCY_SAMPLES = 100
ROS_DOMAIN_ID = "27"
COMPLETION_TOLERANCE_RAD = 1.0e-9

LATENCY_DEFINITION = "command_publish_to_accepted_feedback"
LATENCY_CLAIM = (
    "PROJECT TEST LATENCY",
    "NOT HARD REAL-TIME GUARANTEE",
)

CLAIM = "PROJECT ROS 2 CONTROLLER ADAPTER"
CLAIM_BOUNDARY = (
    "NOT A ROS 2 REAL-TIME CERTIFIED CONTROLLER",
    "NOT ros2_control HARDWARE DRIVER",
    "NOT A UNIVERSAL ROBOTS PRODUCTION DRIVER",
    "NOT A POWER PMAC ROS DRIVER",
    "NOT INDUSTRIAL SAFETY-CERTIFIED COMMUNICATION",
    "NOT REAL ROBOT VALIDATED",
    "PROJECT TEST LATENCY",
    "NOT HARD REAL-TIME GUARANTEE",
)

JOINT_STATE_SOURCE = "https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg"
MESSAGE_CHOICE = (
    "sensor_msgs/JointState carries named joint positions in radians. "
    "/joint_states publishes actual_q only. "
    "/robotwin/target_joint_states publishes target_q only. "
    "command_id, controller_state, fault, and execution_state are not JointState fields. "
    "Those values use a JSON object in std_msgs/String so the ROS message is not the canonical command model. "
    "header.frame_id is not used as command_id."
)

SHUTDOWN_POLICY = (
    "Adapter shutdown calls disconnect() on the controller. "
    "An active MOVE_JOINT is aborted with execution REJECTED and error_code DISCONNECTED. "
    "controller_state becomes DISCONNECTED. "
    "actual joint positions are left where they are. "
    "This is session teardown, not an E-stop and not a safety-certified stop."
)

SUPPORTED_COMMAND = "MOVE_JOINT"
CONTROLLER_BACKEND = "VirtualPmacController"
ADAPTER_IMPLEMENTATION = "Ros2ControllerAdapter"
NODE_NAME = "ros2_controller_adapter"
CLIENT_NODE_NAME = "p007_command_client"
ROBOT_ID = "ur10e"
UNITS = {"angle": "radian", "time": "second"}
