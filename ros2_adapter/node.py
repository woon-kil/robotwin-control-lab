"""Ros2ControllerAdapter.

PROJECT ROS 2 CONTROLLER ADAPTER.
ROS 2는 transport다. slew와 fault 전이는 VirtualPmacController에만 있다.
Isaac, Unity, 카메라는 이 프로세스에서 부르지 않는다.
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from controller.virtual_pmac import ControllerError, VirtualPmacController
from core.model.ur10e import load_ur10e_model
from ros2_adapter.contract import (
    CANONICAL_JOINT_ORDER,
    CONFIGURED_FEEDBACK_HZ,
    CONTROLLER_BACKEND,
    DT_S,
    NODE_NAME,
    QOS_DEPTH,
    ROBOT_ID,
    TOPIC_COMMAND,
    TOPIC_COUNT_WINDOW,
    TOPIC_INJECT_FAULT,
    TOPIC_JOINT_STATES,
    TOPIC_RESET_FAULT,
    TOPIC_RESULT,
    TOPIC_SERVO_ON,
    TOPIC_SHUTDOWN,
    TOPIC_STATE,
    TOPIC_TARGET_JOINT_STATES,
)
from ros2_adapter.convert import actual_positions, decode_command, feedback_document, target_positions


class Ros2ControllerAdapter(Node):
    """Canonical controller 하나를 ROS publisher/subscription 뒤에 둔다."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        model = load_ur10e_model()
        if tuple(model.actuated_joint_order) != CANONICAL_JOINT_ORDER:
            raise RuntimeError("UR10e joint order does not match the P-007 contract")
        self._controller = VirtualPmacController(model)
        self._controller.connect()
        self._controller.servo_on()
        profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._result_pub = self.create_publisher(String, TOPIC_RESULT, profile)
        self._state_pub = self.create_publisher(String, TOPIC_STATE, profile)
        self._joint_pub = self.create_publisher(JointState, TOPIC_JOINT_STATES, profile)
        self._target_pub = self.create_publisher(JointState, TOPIC_TARGET_JOINT_STATES, profile)
        self.create_subscription(String, TOPIC_COMMAND, self._on_command, profile)
        self.create_subscription(String, TOPIC_INJECT_FAULT, self._on_fault, profile)
        self.create_subscription(String, TOPIC_RESET_FAULT, self._on_reset, profile)
        self.create_subscription(String, TOPIC_SERVO_ON, self._on_servo, profile)
        self.create_subscription(String, TOPIC_COUNT_WINDOW, self._on_window, profile)
        self.create_subscription(String, TOPIC_SHUTDOWN, self._on_shutdown, profile)
        self._timer = self.create_timer(1.0 / CONFIGURED_FEEDBACK_HZ, self._on_tick)
        self._feedback_seq = 0
        self._steps = 0
        self._command_start_s = 0.0
        self._active_command_id: str | None = None
        self._window_active = False
        self._window_state_count = 0
        self._window_joint_count = 0
        self._window_started: float | None = None
        self._window_report: dict[str, object] | None = None
        self._pause_feedback = False
        self._shutdown_requested = False
        self._exit_after = 0
        self._closed = False
        self.get_logger().info(CONTROLLER_BACKEND + " READY")

    def shutdown_session(self) -> None:
        """disconnect 한 번. joint를 0으로 옮기지 않는다. E-stop이 아니다."""
        if self._closed:
            return
        self._closed = True
        self._pause_feedback = False
        self._controller.disconnect()
        self._publish_feedback(window="shutdown")

    def _on_tick(self) -> None:
        if self._shutdown_requested and not self._closed:
            self.shutdown_session()
            self._shutdown_requested = False
            self._exit_after = 5
            return
        if self._exit_after:
            self._exit_after -= 1
            if self._exit_after == 0 and rclpy.ok():
                rclpy.shutdown()
            return
        if self._closed or self._pause_feedback:
            return
        active = self._controller.get_state().active_command_id
        state = self._controller.step(DT_S)
        if active is not None:
            self._steps += 1
            if state.execution_state == "COMPLETED" and state.active_command_id is None:
                self._publish_completed(state)
        self._publish_feedback(window=None)

    def _on_command(self, message: String) -> None:
        if self._closed:
            return
        decoded = decode_command(message.data)
        if decoded.error_code is not None or decoded.command is None:
            self._publish_result(
                {
                    "command_id": _command_id_or_blank(message.data),
                    "result": "REJECTED",
                    "error_code": decoded.error_code,
                    "input_joint_names": list(decoded.input_names),
                    "input_positions": list(decoded.input_positions),
                    "canonical_joint_names": list(CANONICAL_JOINT_ORDER),
                    "canonical_target": None,
                }
            )
            return
        result = self._controller.submit(decoded.command)
        self._publish_result(
            {
                "command_id": decoded.command.command_id,
                "result": result.result,
                "error_code": result.error_code,
                "input_joint_names": list(decoded.input_names),
                "input_positions": list(decoded.input_positions),
                "canonical_joint_names": list(CANONICAL_JOINT_ORDER),
                "canonical_target": list(decoded.canonical_positions or ()),
            }
        )
        if result.result == "ACCEPTED":
            self._steps = 0
            self._active_command_id = decoded.command.command_id
            self._command_start_s = self._controller.get_state().timestamp_s
            self._publish_feedback(window=None)

    def _on_fault(self, message: String) -> None:
        self._call_controller(lambda: self._controller.inject_fault(message.data), "inject_fault")

    def _on_reset(self, message: String) -> None:
        del message
        self._call_controller(self._controller.reset_fault, "reset_fault")

    def _on_servo(self, message: String) -> None:
        del message
        self._call_controller(self._controller.servo_on, "servo_on")

    def _on_window(self, message: String) -> None:
        if message.data == "begin":
            self._window_active = False
            self._pause_feedback = False
            self._window_report = None
            self._publish_feedback(window="begin")
            self._window_state_count = 0
            self._window_joint_count = 0
            self._window_active = True
            self._window_started = time.perf_counter()
            return
        if message.data == "end" and self._window_started is not None:
            self._window_active = False
            self._window_report = {
                "published_state": self._window_state_count,
                "published_joint_states": self._window_joint_count,
                "duration_s": time.perf_counter() - self._window_started,
                "configured_rate_hz": CONFIGURED_FEEDBACK_HZ,
            }
            self._pause_feedback = True
            self._publish_feedback(window="stop")

    def _on_shutdown(self, message: String) -> None:
        del message
        self._shutdown_requested = True

    def _call_controller(self, action, name: str) -> None:
        if self._closed:
            return
        try:
            action()
        except ControllerError as exc:
            self._publish_result(
                {
                    "command_id": None,
                    "result": "REJECTED",
                    "error_code": exc.code,
                    "detail": name,
                }
            )
            return
        self._publish_feedback(window=None)

    def _publish_completed(self, state) -> None:
        actual = actual_positions(state)
        target = target_positions(state)
        self._publish_result(
            {
                "command_id": self._active_command_id,
                "result": "COMPLETED",
                "error_code": None,
                "canonical_target": list(target),
                "actual_q": list(actual),
                "max_final_error_rad": max(abs(goal - current) for goal, current in zip(target, actual, strict=True)),
                "steps": self._steps,
                "simulation_time_s": state.timestamp_s,
                "simulated_duration_s": state.timestamp_s - self._command_start_s,
            }
        )

    def _publish_feedback(self, window: str | None) -> None:
        state = self._controller.get_state()
        self._feedback_seq += 1
        document = feedback_document(state, self._feedback_seq)
        if self._window_active:
            document["count_window"] = "open"
            self._window_state_count += 1
        elif window is not None:
            document["count_window"] = window
            if window == "stop" and self._window_report is not None:
                document["window_report"] = self._window_report
        else:
            document["count_window"] = "idle"
        message = String()
        message.data = json.dumps(document, allow_nan=True, sort_keys=True)
        self._state_pub.publish(message)
        if window in ("stop", "begin"):
            return
        self._publish_joint_state(actual_positions(state), state.joint_velocity, include_velocity=True)
        self._publish_joint_state(target_positions(state), None, include_velocity=False, target=True)
        if self._window_active:
            self._window_joint_count += 1

    def _publish_joint_state(self, positions, velocity, include_velocity: bool, target: bool = False) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = ROBOT_ID
        message.name = list(CANONICAL_JOINT_ORDER)
        message.position = [float(value) for value in positions]
        if include_velocity and velocity is not None:
            message.velocity = [float(value) for value in velocity]
        publisher = self._target_pub if target else self._joint_pub
        publisher.publish(message)

    def _publish_result(self, payload: dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(payload, allow_nan=True, sort_keys=True)
        self._result_pub.publish(message)


def _command_id_or_blank(payload_text: str) -> str:
    try:
        payload = json.loads(payload_text)
    except json.JSONDecodeError:
        return ""
    if isinstance(payload, dict) and isinstance(payload.get("command_id"), str):
        return payload["command_id"]
    return ""


def main() -> None:
    rclpy.init()
    node = Ros2ControllerAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_session()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
