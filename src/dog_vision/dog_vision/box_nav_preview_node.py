#!/usr/bin/env python3
"""Preview and optionally execute a simple path to the selected box."""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odometry(msg: Odometry) -> float:
    orientation = msg.pose.pose.orientation
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class NavPlan:
    frame_id: str
    start_x: float
    start_y: float
    target_x: float
    target_y: float
    goal_x: float
    goal_y: float
    goal_yaw: float


class BoxNavPreviewNode(Node):
    def __init__(self) -> None:
        super().__init__('box_nav_preview_node')

        self.declare_parameter('target_pose_topic', '/navigation/target_pose')
        self.declare_parameter('odom_topic', '/ground_truth')
        self.declare_parameter('path_topic', '/navigation/preview_path')
        self.declare_parameter('marker_topic', '/navigation/preview_markers')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('gait_action_topic', '/gait_action')
        self.declare_parameter('status_topic', '/navigation/nav_status')
        self.declare_parameter('publish_cmd_vel', False)
        self.declare_parameter('crouch_on_arrival', False)
        self.declare_parameter('arrival_action', 'crouch')
        self.declare_parameter('control_mode', 'odom')
        self.declare_parameter('drive_mode', 'turn_then_forward')
        self.declare_parameter('enable_yaw_control', True)
        self.declare_parameter('use_robot_pose', True)
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('target_frame_mode', 'auto')
        self.declare_parameter('lock_target', False)
        self.declare_parameter('sensor_forward_offset', 0.0)
        self.declare_parameter('standoff_distance', 0.70)
        self.declare_parameter('replan_distance', 0.08)
        self.declare_parameter('target_timeout', 0.8)
        self.declare_parameter('timer_period', 0.1)
        self.declare_parameter('linear_tolerance', 0.08)
        self.declare_parameter('yaw_tolerance', 0.08)
        self.declare_parameter('linear_gain', 0.45)
        self.declare_parameter('lateral_gain', 0.45)
        self.declare_parameter('angular_gain', 1.2)
        self.declare_parameter('min_linear_speed', 0.04)
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('min_lateral_speed', 0.04)
        self.declare_parameter('max_lateral_speed', 0.10)
        self.declare_parameter('min_angular_speed', 0.12)
        self.declare_parameter('max_angular_speed', 0.35)

        self.target_pose_topic = str(self.get_parameter('target_pose_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.gait_action_topic = str(self.get_parameter('gait_action_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.publish_cmd_vel = bool(self.get_parameter('publish_cmd_vel').value)
        self.crouch_on_arrival = bool(
            self.get_parameter('crouch_on_arrival').value
        )
        self.arrival_action = str(self.get_parameter('arrival_action').value)
        self.control_mode = str(self.get_parameter('control_mode').value).lower()
        if self.control_mode not in ('odom', 'relative'):
            self.get_logger().warn(
                "Unknown control_mode '%s', using odom." % self.control_mode
            )
            self.control_mode = 'odom'
        self.drive_mode = str(self.get_parameter('drive_mode').value).lower()
        if self.drive_mode not in ('turn_then_forward', 'holonomic'):
            self.get_logger().warn(
                "Unknown drive_mode '%s', using turn_then_forward."
                % self.drive_mode
            )
            self.drive_mode = 'turn_then_forward'
        self.enable_yaw_control = bool(
            self.get_parameter('enable_yaw_control').value
        )
        self.use_robot_pose = bool(self.get_parameter('use_robot_pose').value)
        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.target_frame_mode = str(
            self.get_parameter('target_frame_mode').value
        ).lower()
        if self.target_frame_mode not in ('auto', 'local', 'fixed'):
            self.get_logger().warn(
                "Unknown target_frame_mode '%s', using auto."
                % self.target_frame_mode
            )
            self.target_frame_mode = 'auto'
        self.lock_target = bool(self.get_parameter('lock_target').value)
        self.sensor_forward_offset = float(
            self.get_parameter('sensor_forward_offset').value
        )
        self.standoff_distance = float(self.get_parameter('standoff_distance').value)
        self.replan_distance = float(self.get_parameter('replan_distance').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.timer_period = float(self.get_parameter('timer_period').value)
        self.linear_tolerance = float(self.get_parameter('linear_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.lateral_gain = float(self.get_parameter('lateral_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_lateral_speed = float(self.get_parameter('min_lateral_speed').value)
        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.gait_action_pub = self.create_publisher(
            String, self.gait_action_topic, 10
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.target_sub = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.target_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.pose: RobotPose | None = None
        self.last_target: PoseStamped | None = None
        self.last_target_time = None
        self.plan: NavPlan | None = None
        self.arrived = False
        self.arrival_action_sent = False
        self.last_log_time = self.get_clock().now()
        self.last_status = 'waiting_for_target'

        mode = 'will publish /cmd_vel' if self.publish_cmd_vel else 'visual preview only'
        self.get_logger().info(
            'Box nav preview started: target=%s, odom=%s, path=%s, target_frame_mode=%s, control_mode=%s, drive_mode=%s, lock_target=%s, crouch_on_arrival=%s, %s'
            % (
                self.target_pose_topic,
                self.odom_topic,
                self.path_topic,
                self.target_frame_mode,
                self.control_mode,
                self.drive_mode,
                'true' if self.lock_target else 'false',
                'true' if self.crouch_on_arrival else 'false',
                mode,
            )
        )

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.pose = RobotPose(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_odometry(msg),
        )

    def target_callback(self, msg: PoseStamped) -> None:
        if self.arrived:
            return
        self.last_target_time = self.get_clock().now()
        if self.lock_target and self.plan is not None:
            return
        if self.plan is None or self.should_replan(msg):
            self.last_target = msg
            self.plan = self.make_plan(msg)
            self.publish_plan(self.plan)
            self.log_plan(self.plan)

    def should_replan(self, msg: PoseStamped) -> bool:
        if self.last_target is None:
            return True
        dx = msg.pose.position.x - self.last_target.pose.position.x
        dy = msg.pose.position.y - self.last_target.pose.position.y
        return math.hypot(dx, dy) >= self.replan_distance

    def make_plan(self, target: PoseStamped) -> NavPlan:
        target_x = float(target.pose.position.x)
        target_y = float(target.pose.position.y)
        if self.use_robot_pose and self.pose is not None:
            start_x = self.pose.x
            start_y = self.pose.y
        else:
            start_x = 0.0
            start_y = 0.0

        target_frame = target.header.frame_id or self.fixed_frame
        frame_id = target_frame
        treat_as_local = self.target_frame_mode == 'local'
        if self.target_frame_mode == 'auto':
            treat_as_local = target_frame != self.fixed_frame

        if treat_as_local and self.pose is not None and self.control_mode == 'odom':
            target_x, target_y = self.local_target_to_fixed(target_x, target_y)
            frame_id = self.fixed_frame
        elif self.target_frame_mode == 'fixed':
            frame_id = self.fixed_frame

        dx = target_x - start_x
        dy = target_y - start_y
        heading = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else 0.0
        goal_x = target_x - math.cos(heading) * self.standoff_distance
        goal_y = target_y - math.sin(heading) * self.standoff_distance

        return NavPlan(
            frame_id=frame_id,
            start_x=start_x,
            start_y=start_y,
            target_x=target_x,
            target_y=target_y,
            goal_x=goal_x,
            goal_y=goal_y,
            goal_yaw=heading,
        )

    def local_target_to_fixed(self, local_x: float, local_y: float) -> tuple[float, float]:
        if self.pose is None:
            return local_x, local_y
        local_x += self.sensor_forward_offset
        cos_yaw = math.cos(self.pose.yaw)
        sin_yaw = math.sin(self.pose.yaw)
        return (
            self.pose.x + cos_yaw * local_x - sin_yaw * local_y,
            self.pose.y + sin_yaw * local_x + cos_yaw * local_y,
        )

    def publish_plan(self, plan: NavPlan) -> None:
        now = self.get_clock().now().to_msg()

        path = Path()
        path.header.stamp = now
        path.header.frame_id = plan.frame_id
        path.poses = [
            self.pose_msg(plan.frame_id, now, plan.start_x, plan.start_y, 0.0),
            self.pose_msg(plan.frame_id, now, plan.goal_x, plan.goal_y, plan.goal_yaw),
        ]
        self.path_pub.publish(path)

        markers = MarkerArray()
        markers.markers.append(
            self.sphere_marker(0, plan.frame_id, now, plan.target_x, plan.target_y)
        )
        markers.markers.append(
            self.arrow_marker(1, plan.frame_id, now, plan.goal_x, plan.goal_y, plan.goal_yaw)
        )
        markers.markers.append(self.line_marker(2, plan.frame_id, now, plan))
        self.marker_pub.publish(markers)

    def pose_msg(
        self,
        frame_id: str,
        stamp: object,
        x: float,
        y: float,
        yaw: float,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.02
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def sphere_marker(
        self,
        marker_id: int,
        frame_id: str,
        stamp: object,
        x: float,
        y: float,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = 'box_nav_preview'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.18
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.05
        marker.color.a = 0.95
        return marker

    def arrow_marker(
        self,
        marker_id: int,
        frame_id: str,
        stamp: object,
        x: float,
        y: float,
        yaw: float,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = 'box_nav_preview'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.08
        marker.pose.orientation.z = math.sin(yaw * 0.5)
        marker.pose.orientation.w = math.cos(yaw * 0.5)
        marker.scale.x = 0.34
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.r = 0.1
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 0.95
        return marker

    def line_marker(
        self,
        marker_id: int,
        frame_id: str,
        stamp: object,
        plan: NavPlan,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = 'box_nav_preview'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 0.35
        marker.color.a = 0.95
        marker.points = [
            self.point(plan.start_x, plan.start_y, 0.04),
            self.point(plan.goal_x, plan.goal_y, 0.04),
            self.point(plan.target_x, plan.target_y, 0.04),
        ]
        return marker

    def point(self, x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def timer_callback(self) -> None:
        if self.plan is None:
            if self.publish_cmd_vel:
                self.cmd_pub.publish(Twist())
            self.publish_status('waiting_for_target')
            return

        self.publish_plan(self.plan)
        if self.arrived:
            if self.publish_cmd_vel:
                self.cmd_pub.publish(Twist())
            self.publish_status('arrived_hold')
            return

        if self.target_timed_out():
            if self.publish_cmd_vel:
                self.cmd_pub.publish(Twist())
            self.publish_status('target_timeout_stop', self.plan, Twist())
            return

        if not self.publish_cmd_vel:
            self.publish_status('preview_only')
            return

        cmd = self.command_for_plan(self.plan)
        self.cmd_pub.publish(cmd)
        self.log_command(self.plan, cmd)

    def command_for_plan(self, plan: NavPlan) -> Twist:
        if self.control_mode == 'relative':
            return self.command_for_relative_plan(plan)
        if self.drive_mode == 'holonomic':
            return self.command_for_holonomic_plan(plan)

        cmd = Twist()
        if self.pose is None:
            self.publish_status('waiting_for_odom')
            return cmd

        dx = plan.goal_x - self.pose.x
        dy = plan.goal_y - self.pose.y
        distance = math.hypot(dx, dy)
        yaw_to_goal = math.atan2(dy, dx) if distance > 1e-6 else plan.goal_yaw
        yaw_error = normalize_angle(yaw_to_goal - self.pose.yaw)

        if distance > self.linear_tolerance:
            if self.enable_yaw_control and abs(yaw_error) > self.yaw_tolerance:
                cmd.angular.z = self.angular_command(yaw_error)
                self.publish_status(
                    'rotate_to_goal',
                    plan,
                    cmd,
                    distance=distance,
                    yaw_error=yaw_error,
                )
                return cmd
            cmd.linear.x = clamp(
                self.linear_gain * distance,
                self.min_linear_speed,
                self.max_linear_speed,
            )
            self.publish_status(
                'drive_forward',
                plan,
                cmd,
                distance=distance,
                yaw_error=yaw_error,
            )
            return cmd

        final_yaw_error = normalize_angle(plan.goal_yaw - self.pose.yaw)
        if self.enable_yaw_control and abs(final_yaw_error) > self.yaw_tolerance:
            cmd.angular.z = self.angular_command(final_yaw_error)
            self.publish_status(
                'align_at_goal',
                plan,
                cmd,
                distance=distance,
                yaw_error=final_yaw_error,
            )
            return cmd

        self.handle_arrival(plan)
        self.publish_status('arrived', plan, cmd, distance=distance, yaw_error=0.0)
        return cmd

    def command_for_holonomic_plan(self, plan: NavPlan) -> Twist:
        cmd = Twist()
        if self.pose is None:
            self.publish_status('waiting_for_odom')
            return cmd

        dx = plan.goal_x - self.pose.x
        dy = plan.goal_y - self.pose.y
        distance = math.hypot(dx, dy)
        final_yaw_error = normalize_angle(plan.goal_yaw - self.pose.yaw)

        if distance > self.linear_tolerance:
            body_x, body_y = self.fixed_error_to_body(dx, dy)
            cmd.linear.x = self.axis_command(
                body_x,
                self.linear_gain,
                self.min_linear_speed,
                self.max_linear_speed,
            )
            cmd.linear.y = self.axis_command(
                body_y,
                self.lateral_gain,
                self.min_lateral_speed,
                self.max_lateral_speed,
            )
            if self.enable_yaw_control and abs(final_yaw_error) > self.yaw_tolerance:
                cmd.angular.z = self.angular_command(final_yaw_error)
            self.publish_status(
                'holonomic_drive',
                plan,
                cmd,
                distance=distance,
                yaw_error=final_yaw_error,
            )
            return cmd

        if self.enable_yaw_control and abs(final_yaw_error) > self.yaw_tolerance:
            cmd.angular.z = self.angular_command(final_yaw_error)
            self.publish_status(
                'align_at_goal',
                plan,
                cmd,
                distance=distance,
                yaw_error=final_yaw_error,
            )
            return cmd

        self.handle_arrival(plan)
        self.publish_status('arrived', plan, cmd, distance=distance, yaw_error=0.0)
        return cmd

    def command_for_relative_plan(self, plan: NavPlan) -> Twist:
        cmd = Twist()
        goal_x, goal_y, target_x, target_y = self.relative_control_geometry(plan)
        distance_to_stop = math.hypot(goal_x, goal_y)
        yaw_error = math.atan2(target_y, max(target_x, 1e-6))

        if self.drive_mode == 'holonomic':
            if distance_to_stop > self.linear_tolerance:
                cmd.linear.x = self.axis_command(
                    goal_x,
                    self.linear_gain,
                    self.min_linear_speed,
                    self.max_linear_speed,
                )
                cmd.linear.y = self.axis_command(
                    goal_y,
                    self.lateral_gain,
                    self.min_lateral_speed,
                    self.max_lateral_speed,
                )
                if self.enable_yaw_control and abs(yaw_error) > self.yaw_tolerance:
                    cmd.angular.z = self.angular_command(yaw_error)
                self.publish_status(
                    'relative_holonomic_drive',
                    plan,
                    cmd,
                    distance=distance_to_stop,
                    yaw_error=yaw_error,
                )
                return cmd

            if self.enable_yaw_control and abs(yaw_error) > self.yaw_tolerance:
                cmd.angular.z = self.angular_command(yaw_error)
                self.publish_status(
                    'relative_align_at_goal',
                    plan,
                    cmd,
                    distance=distance_to_stop,
                    yaw_error=yaw_error,
                )
                return cmd

            self.handle_arrival(plan)
            self.publish_status(
                'arrived', plan, cmd, distance=distance_to_stop, yaw_error=0.0
            )
            return cmd

        if distance_to_stop > self.linear_tolerance:
            if self.enable_yaw_control and abs(yaw_error) > self.yaw_tolerance:
                cmd.angular.z = self.angular_command(yaw_error)
                self.publish_status(
                    'relative_rotate_to_goal',
                    plan,
                    cmd,
                    distance=distance_to_stop,
                    yaw_error=yaw_error,
                )
                return cmd
            cmd.linear.x = clamp(
                self.linear_gain * distance_to_stop,
                self.min_linear_speed,
                self.max_linear_speed,
            )
            self.publish_status(
                'relative_drive_forward',
                plan,
                cmd,
                distance=distance_to_stop,
                yaw_error=yaw_error,
            )
            return cmd

        if self.enable_yaw_control and abs(yaw_error) > self.yaw_tolerance:
            cmd.angular.z = self.angular_command(yaw_error)
            self.publish_status(
                'relative_align_at_goal',
                plan,
                cmd,
                distance=distance_to_stop,
                yaw_error=yaw_error,
            )
            return cmd

        self.handle_arrival(plan)
        self.publish_status('arrived', plan, cmd, distance=distance_to_stop, yaw_error=0.0)
        return cmd

    def handle_arrival(self, plan: NavPlan) -> None:
        if self.arrived:
            return
        self.arrived = True
        self.cmd_pub.publish(Twist())
        self.get_logger().info(
            'Arrived at stop pose near box(%.2f, %.2f).'
            % (plan.target_x, plan.target_y)
        )
        if self.crouch_on_arrival:
            self.send_arrival_action()

    def send_arrival_action(self) -> None:
        if self.arrival_action_sent:
            return
        msg = String()
        msg.data = self.arrival_action
        self.gait_action_pub.publish(msg)
        self.arrival_action_sent = True
        self.get_logger().info(
            "Arrival gait action -> %s, navigation held in arrived state."
            % self.arrival_action
        )

    def angular_command(self, yaw_error: float) -> float:
        if abs(yaw_error) <= self.yaw_tolerance:
            return 0.0
        angular = clamp(
            self.angular_gain * yaw_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if abs(angular) < self.min_angular_speed:
            angular = math.copysign(self.min_angular_speed, angular)
        return angular

    def axis_command(
        self,
        error: float,
        gain: float,
        min_speed: float,
        max_speed: float,
    ) -> float:
        if abs(error) <= self.linear_tolerance:
            return 0.0
        speed = clamp(gain * error, -max_speed, max_speed)
        if 0.0 < abs(speed) < min_speed:
            speed = math.copysign(min_speed, speed)
        return speed

    def fixed_error_to_body(self, dx: float, dy: float) -> tuple[float, float]:
        if self.pose is None:
            return dx, dy
        cos_yaw = math.cos(self.pose.yaw)
        sin_yaw = math.sin(self.pose.yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
        )

    def relative_control_geometry(
        self, plan: NavPlan
    ) -> tuple[float, float, float, float]:
        target_x = plan.target_x + self.sensor_forward_offset
        target_y = plan.target_y
        distance = math.hypot(target_x, target_y)
        if distance <= 1e-6:
            return 0.0, 0.0, target_x, target_y
        goal_x = target_x - (target_x / distance) * self.standoff_distance
        goal_y = target_y - (target_y / distance) * self.standoff_distance
        return goal_x, goal_y, target_x, target_y

    def target_age_seconds(self) -> float | None:
        if self.last_target_time is None:
            return None
        return (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9

    def target_timed_out(self) -> bool:
        if self.target_timeout <= 0.0:
            return False
        target_age = self.target_age_seconds()
        return target_age is not None and target_age > self.target_timeout

    def publish_status(
        self,
        state: str,
        plan: NavPlan | None = None,
        cmd: Twist | None = None,
        *,
        distance: float | None = None,
        yaw_error: float | None = None,
    ) -> None:
        parts = [
            'state=%s' % state,
            'control_mode=%s' % self.control_mode,
            'drive_mode=%s' % self.drive_mode,
            'enable_yaw_control=%s' % ('true' if self.enable_yaw_control else 'false'),
            'publish_cmd_vel=%s' % ('true' if self.publish_cmd_vel else 'false'),
        ]
        target_age = self.target_age_seconds()
        if target_age is not None:
            parts.append('target_age=%.2fs' % target_age)
        if distance is not None:
            parts.append('distance=%.3f' % distance)
        if yaw_error is not None:
            parts.append('yaw_error=%.1fdeg' % math.degrees(yaw_error))
        if plan is not None:
            parts.append(
                'goal=(%.2f, %.2f)' % (plan.goal_x, plan.goal_y)
            )
            parts.append(
                'target=(%.2f, %.2f)' % (plan.target_x, plan.target_y)
            )
        if cmd is not None:
            parts.append(
                'cmd_vel=(x=%.3f, y=%.3f, yaw=%.3f)'
                % (cmd.linear.x, cmd.linear.y, cmd.angular.z)
            )

        status = String()
        status.data = ', '.join(parts)
        self.last_status = status.data
        self.status_pub.publish(status)

    def log_plan(self, plan: NavPlan) -> None:
        self.get_logger().info(
            'Preview path: start(%.2f, %.2f) -> stop(%.2f, %.2f) -> box(%.2f, %.2f), yaw %.1fdeg'
            % (
                plan.start_x,
                plan.start_y,
                plan.goal_x,
                plan.goal_y,
                plan.target_x,
                plan.target_y,
                math.degrees(plan.goal_yaw),
            )
        )

    def log_command(self, plan: NavPlan, cmd: Twist) -> None:
        if self.pose is None:
            return
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 < 1.0:
            return
        self.last_log_time = now
        dist = math.hypot(plan.goal_x - self.pose.x, plan.goal_y - self.pose.y)
        self.get_logger().info(
            'Driving preview path: dist %.2fm, cmd vx %.2f vy %.2f wz %.2f'
            % (dist, cmd.linear.x, cmd.linear.y, cmd.angular.z)
        )


def main() -> None:
    rclpy.init()
    node = BoxNavPreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
