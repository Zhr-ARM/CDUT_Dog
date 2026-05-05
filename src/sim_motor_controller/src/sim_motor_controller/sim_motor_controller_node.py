"""Simulation-side MIT motor controller bridge.

This node keeps the same MIT command surface as the real motor controller, but
drives Gazebo through ros2_control. The default simulation backend sends direct
joint position commands, while an effort-output mode is kept for experiments.
"""

import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
from std_srvs.srv import Trigger

# Reuse the canonical MIT types from the real-motor protocol package.
from real_motor_controller.dm_protocol import MITCommand, mit_command_expired  # noqa: E402
from real_motor_controller.ros_params import declare_parameters, load_parameters  # noqa: E402

ZERO_MIT_COMMAND = MITCommand(0.0, 0.0, 0.0, 0.0, 0.0)


PARAM_DEFAULTS = {
    "motor_ids": [1, 2, 3, 4],
    "joint_names": [
        "base_yaw_joint",
        "shoulder_pitch_joint",
        "elbow_pitch_joint",
        "wrist_pitch_joint",
    ],
    "mit_command_topic": "/dm_j4340/mit_commands",
    "joint_states_topic": "/joint_states",
    "dm_joint_states_topic": "/dm_j4340/joint_states",
    "command_output_mode": "position",
    "position_command_topic": "/arm_position_controller/commands",
    "effort_command_topic": "/arm_effort_controller/commands",
    "status_topic": "/dm_j4340/status",
    "auto_enable_on_start": True,
    "mit_command_timeout_s": 0.2,
    "state_timeout_s": 0.5,
    "command_rate_hz": 200.0,
    "state_rate_hz": 50.0,
    "status_rate_hz": 5.0,
    "p_min": -12.5,
    "p_max": 12.5,
    "v_min": -30.0,
    "v_max": 30.0,
    "t_min": -10.0,
    "t_max": 10.0,
}


class SimMotorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_motor_controller_node")
        self._declare_parameters()
        self._load_parameters()
        self._validate_config()

        count = len(self.motor_ids)
        self.enabled = bool(self.auto_enable_on_start)
        self.positions = [0.0] * count
        self.velocities = [0.0] * count
        self.efforts = [0.0] * count
        self.zero_offsets = [0.0] * count
        self.commands = [ZERO_MIT_COMMAND for _ in range(count)]
        self.last_command_time: Optional[float] = None
        self.last_joint_state_time: Optional[float] = None
        self.last_published_command = [0.0] * count
        self.controller_command_topic = (
            self.position_command_topic
            if self.command_output_mode == "position"
            else self.effort_command_topic
        )

        self.dm_state_publisher = self.create_publisher(
            JointState, self.dm_joint_states_topic, 10
        )
        self.controller_command_publisher = self.create_publisher(
            Float64MultiArray, self.controller_command_topic, 10
        )
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)

        self.joint_state_subscriber = self.create_subscription(
            JointState, self.joint_states_topic, self._handle_joint_state, 10
        )
        self.command_subscriber = self.create_subscription(
            Float32MultiArray,
            self.mit_command_topic,
            self._handle_mit_command,
            10,
        )

        self.enable_service = self.create_service(
            Trigger, "/dm_j4340/enable_all", self._handle_enable_all
        )
        self.disable_service = self.create_service(
            Trigger, "/dm_j4340/disable_all", self._handle_disable_all
        )
        self.zero_service = self.create_service(
            Trigger, "/dm_j4340/set_zero_all", self._handle_set_zero_all
        )
        self.clear_fault_service = self.create_service(
            Trigger, "/dm_j4340/clear_faults_all", self._handle_clear_faults_all
        )

        self.command_timer = self.create_timer(
            1.0 / self.command_rate_hz, self._publish_controller_command
        )
        self.state_timer = self.create_timer(1.0 / self.state_rate_hz, self._publish_joint_state)
        self.status_timer = self.create_timer(1.0 / self.status_rate_hz, self._publish_status)

        self.get_logger().info(
            "Started simulated MIT controller bridge on "
            f"{self.mit_command_topic} -> {self.controller_command_topic} "
            f"mode={self.command_output_mode} "
            f"for motor_ids={self.motor_ids}, joints={self.joint_names}."
        )

    def _declare_parameters(self) -> None:
        declare_parameters(self, PARAM_DEFAULTS)

    def _load_parameters(self) -> None:
        self.__dict__.update(load_parameters(self, PARAM_DEFAULTS))

    def _validate_config(self) -> None:
        self.command_output_mode = str(self.command_output_mode).strip().lower()
        if self.command_output_mode not in ("position", "effort"):
            raise ValueError("command_output_mode must be either 'position' or 'effort'.")

        count = len(self.motor_ids)
        for name in ("joint_names",):
            if len(getattr(self, name)) != count:
                raise ValueError(f"{name} length must match motor_ids length.")
        for name in (
            "command_rate_hz",
            "state_rate_hz",
            "status_rate_hz",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be greater than zero.")
        for name in ("mit_command_timeout_s", "state_timeout_s"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be greater than or equal to zero.")

    def _handle_joint_state(self, msg: JointState) -> None:
        index_by_name = {name: index for index, name in enumerate(msg.name)}
        for index, joint_name in enumerate(self.joint_names):
            joint_index = index_by_name.get(joint_name)
            if joint_index is None:
                continue
            self.positions[index] = self._extract_float(msg.position, joint_index)
            self.velocities[index] = self._extract_float(msg.velocity, joint_index)
            self.efforts[index] = self._extract_float(msg.effort, joint_index)
        self.last_joint_state_time = time.monotonic()

    def _handle_mit_command(self, msg: Float32MultiArray) -> None:
        values = [float(value) for value in msg.data]
        expected_length = len(self.motor_ids) * 5
        if len(values) != expected_length:
            self.get_logger().warning(
                f"Expected {expected_length} MIT command values "
                f"([p,v,kp,kd,t_ff] * {len(self.motor_ids)}), got {len(values)}."
            )
            return

        self.commands = [
            MITCommand(
                position=self._clamp(values[index], self.p_min, self.p_max),
                velocity=self._clamp(values[index + 1], self.v_min, self.v_max),
                kp=max(0.0, values[index + 2]),
                kd=max(0.0, values[index + 3]),
                torque=self._clamp(values[index + 4], self.t_min, self.t_max),
            )
            for index in range(0, expected_length, 5)
        ]
        self.last_command_time = time.monotonic()

    def _current_commands(self) -> List[MITCommand]:
        if not self.enabled:
            return [ZERO_MIT_COMMAND for _ in self.motor_ids]
        if mit_command_expired(self.last_command_time, self.mit_command_timeout_s):
            return [ZERO_MIT_COMMAND for _ in self.motor_ids]
        return list(self.commands)

    def _has_recent_mit_command(self) -> bool:
        return not mit_command_expired(self.last_command_time, self.mit_command_timeout_s)

    def _has_fresh_joint_state(self) -> bool:
        if self.last_joint_state_time is None:
            return False
        if self.state_timeout_s <= 0.0:
            return True
        return time.monotonic() - self.last_joint_state_time <= self.state_timeout_s

    def _current_relative_positions(self) -> List[float]:
        return [position - offset for position, offset in zip(self.positions, self.zero_offsets)]

    def _current_effort_command(self) -> List[float]:
        if not self.enabled or not self._has_fresh_joint_state():
            return [0.0] * len(self.motor_ids)

        commands = self._current_commands()
        efforts: List[float] = []
        for index, command in enumerate(commands):
            position_error = command.position - (
                self.positions[index] - self.zero_offsets[index]
            )
            velocity_error = command.velocity - self.velocities[index]
            effort = command.kp * position_error + command.kd * velocity_error + command.torque
            efforts.append(self._clamp(effort, self.t_min, self.t_max))
        return efforts

    def _current_position_command(self) -> List[float]:
        if not self.enabled or not self._has_fresh_joint_state():
            return list(self.positions)
        if not self._has_recent_mit_command():
            return list(self.positions)

        commands: List[float] = []
        for index, command in enumerate(self.commands):
            relative_position = self._clamp(command.position, self.p_min, self.p_max)
            commands.append(relative_position + self.zero_offsets[index])
        return commands

    def _publish_controller_command(self) -> None:
        if self.command_output_mode == "position":
            command_values = self._current_position_command()
        else:
            command_values = self._current_effort_command()
        self.last_published_command = list(command_values)

        msg = Float64MultiArray()
        msg.data = command_values
        self.controller_command_publisher.publish(msg)

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = self._current_relative_positions()
        msg.velocity = list(self.velocities)
        msg.effort = list(self.efforts)
        self.dm_state_publisher.publish(msg)

    def _publish_status(self) -> None:
        if not self.enabled:
            mode = "disabled"
        elif not self._has_fresh_joint_state():
            mode = "waiting_joint_states"
        elif self.last_command_time is None:
            mode = "waiting_command"
        elif self.mit_command_timeout_s > 0.0 and (
            time.monotonic() - self.last_command_time > self.mit_command_timeout_s
        ):
            mode = "holding_position" if self.command_output_mode == "position" else "zero_mit"
        else:
            mode = "topic_mit"

        state_age = self._age_seconds(self.last_joint_state_time)
        command_age = self._age_seconds(self.last_command_time)
        positions = ",".join(f"{value:.3f}" for value in self._current_relative_positions())
        command_values = ",".join(f"{value:.3f}" for value in self.last_published_command)

        msg = String()
        msg.data = (
            f"enabled={int(self.enabled)} mode={mode} "
            f"output={self.command_output_mode} "
            f"state_age={state_age:.3f} command_age={command_age:.3f} "
            f"positions=[{positions}] command=[{command_values}] sim=1"
        )
        self.status_publisher.publish(msg)

    def _handle_enable_all(self, _request, response):
        self.enabled = True
        response.success = True
        response.message = "Simulated motors enabled."
        return response

    def _handle_disable_all(self, _request, response):
        self.enabled = False
        response.success = True
        response.message = "Simulated motors disabled."
        return response

    def _handle_set_zero_all(self, _request, response):
        self.zero_offsets = list(self.positions)
        response.success = True
        response.message = "Captured current joint positions as the new zero offsets."
        return response

    def _handle_clear_faults_all(self, _request, response):
        response.success = True
        response.message = "Simulated motors have no faults."
        return response

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return min(max(value, low), high)

    @staticmethod
    def _extract_float(values, index: int) -> float:
        if index >= len(values):
            return 0.0
        return float(values[index])

    @staticmethod
    def _age_seconds(timestamp: Optional[float]) -> float:
        if timestamp is None:
            return -1.0
        return time.monotonic() - timestamp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SimMotorControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
