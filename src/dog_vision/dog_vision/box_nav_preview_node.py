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
    source_frame_id: str
    raw_target_x: float
    raw_target_y: float
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
        self.declare_parameter('gait_status_topic', '/gait_status')
        self.declare_parameter('status_topic', '/navigation/nav_status')
        self.declare_parameter('debug_status_topic', '/navigation/nav_debug_zh')
        self.declare_parameter('nav_debug_log', True)
        self.declare_parameter('nav_debug_log_period', 0.5)
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
        self.declare_parameter('orthogonal_lateral_tolerance', 0.15)
        self.declare_parameter('orthogonal_yaw_tolerance', 0.20)
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
        self.gait_status_topic = str(self.get_parameter('gait_status_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.debug_status_topic = str(self.get_parameter('debug_status_topic').value)
        self.nav_debug_log = bool(self.get_parameter('nav_debug_log').value)
        self.nav_debug_log_period = float(
            self.get_parameter('nav_debug_log_period').value
        )
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
        if self.drive_mode not in ('turn_then_forward', 'holonomic', 'orthogonal'):
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
        self.orthogonal_lateral_tolerance = float(
            self.get_parameter('orthogonal_lateral_tolerance').value
        )
        self.orthogonal_yaw_tolerance = float(
            self.get_parameter('orthogonal_yaw_tolerance').value
        )
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
        self.debug_status_pub = self.create_publisher(
            String, self.debug_status_topic, 10
        )
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
        self.gait_status_sub = self.create_subscription(
            String,
            self.gait_status_topic,
            self.gait_status_callback,
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
        self.last_debug_log_time = self.get_clock().now()
        self.last_debug_state = ''
        self.last_status = 'waiting_for_target'
        self.latest_gait_status = ''

        mode = 'will publish /cmd_vel' if self.publish_cmd_vel else 'visual preview only'
        self.get_logger().info(
            '箱子导航启动: target=%s, odom=%s, path=%s, target_frame_mode=%s, control_mode=%s, drive_mode=%s, lock_target=%s, crouch_on_arrival=%s, %s'
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
        if self.plan is None and self.last_target is not None and not self.arrived:
            self.plan = self.make_plan(self.last_target)
            self.publish_plan(self.plan)
            self.log_plan(self.plan)

    def gait_status_callback(self, msg: String) -> None:
        self.latest_gait_status = msg.data

    def target_callback(self, msg: PoseStamped) -> None:
        if self.arrived:
            return
        self.last_target_time = self.get_clock().now()
        if self.needs_odom_projection(msg) and self.pose is None:
            self.last_target = msg
            self.publish_status(
                'waiting_for_odom_projection',
                action='等待',
                reason='目标是相机局部坐标，正在等待 odom 后再锁定停车点',
            )
            return
        if self.lock_target and self.plan is not None:
            return
        if self.plan is None or self.should_replan(msg):
            self.last_target = msg
            self.plan = self.make_plan(msg)
            self.publish_plan(self.plan)
            self.log_plan(self.plan)

    def needs_odom_projection(self, target: PoseStamped) -> bool:
        return self.control_mode == 'odom' and self.target_is_local(target)

    def target_is_local(self, target: PoseStamped) -> bool:
        target_frame = target.header.frame_id or self.fixed_frame
        if self.target_frame_mode == 'local':
            return True
        if self.target_frame_mode == 'auto':
            return target_frame != self.fixed_frame
        return False

    def should_replan(self, msg: PoseStamped) -> bool:
        if self.last_target is None:
            return True
        dx = msg.pose.position.x - self.last_target.pose.position.x
        dy = msg.pose.position.y - self.last_target.pose.position.y
        return math.hypot(dx, dy) >= self.replan_distance

    def make_plan(self, target: PoseStamped) -> NavPlan:
        raw_target_x = float(target.pose.position.x)
        raw_target_y = float(target.pose.position.y)
        target_x = raw_target_x
        target_y = raw_target_y
        if self.use_robot_pose and self.pose is not None:
            start_x = self.pose.x
            start_y = self.pose.y
        else:
            start_x = 0.0
            start_y = 0.0

        target_frame = target.header.frame_id or self.fixed_frame
        frame_id = target_frame
        treat_as_local = self.target_is_local(target)

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
            source_frame_id=target_frame,
            raw_target_x=raw_target_x,
            raw_target_y=raw_target_y,
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
            if (
                self.last_target is not None
                and self.needs_odom_projection(self.last_target)
                and self.pose is None
            ):
                self.publish_status(
                    'waiting_for_odom_projection',
                    action='等待',
                    reason='已收到相机目标，正在等待 odom 后再锁定停车点',
                )
                return
            self.publish_status(
                'waiting_for_target',
                action='等待',
                reason='还没有收到箱子目标',
            )
            return

        self.publish_plan(self.plan)
        if self.arrived:
            if self.publish_cmd_vel:
                self.cmd_pub.publish(Twist())
            self.publish_status(
                'arrived_hold',
                self.plan,
                Twist(),
                action='停止',
                reason='已经到达，保持停车',
            )
            return

        if self.target_timed_out():
            if self.publish_cmd_vel:
                self.cmd_pub.publish(Twist())
            self.publish_status(
                'target_timeout_stop',
                self.plan,
                Twist(),
                action='停止',
                reason='目标数据超时，安全停车',
            )
            return

        if not self.publish_cmd_vel:
            self.publish_status(
                'preview_only',
                self.plan,
                action='预览',
                reason='publish_cmd_vel=false，只计算不发速度',
            )
            return

        cmd = self.command_for_plan(self.plan)
        self.cmd_pub.publish(cmd)
        self.log_command(self.plan, cmd)

    def command_for_plan(self, plan: NavPlan) -> Twist:
        if self.control_mode == 'relative':
            return self.command_for_relative_plan(plan)
        if self.drive_mode == 'holonomic':
            return self.command_for_holonomic_plan(plan)
        if self.drive_mode == 'orthogonal':
            return self.command_for_orthogonal_plan(plan)

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

    def command_for_orthogonal_plan(self, plan: NavPlan) -> Twist:
        cmd = Twist()
        if self.pose is None:
            self.publish_status('waiting_for_odom')
            return cmd

        dx = plan.goal_x - self.pose.x
        dy = plan.goal_y - self.pose.y
        distance = math.hypot(dx, dy)
        body_x, body_y = self.fixed_error_to_body(dx, dy)
        final_yaw_error = normalize_angle(plan.goal_yaw - self.pose.yaw)

        if abs(body_y) > self.orthogonal_lateral_tolerance:
            cmd.linear.y = self.axis_command(
                body_y,
                self.lateral_gain,
                self.min_lateral_speed,
                self.max_lateral_speed,
            )
            self.publish_status(
                'orthogonal_lateral_align',
                plan,
                cmd,
                distance=distance,
                yaw_error=final_yaw_error,
                body_x=body_x,
                body_y=body_y,
                action=self.movement_name(cmd),
                reason=(
                    'body_y %.2fm 超过横向启动阈值 %.2fm'
                    % (body_y, self.orthogonal_lateral_tolerance)
                ),
            )
            return cmd

        if (
            self.enable_yaw_control
            and abs(final_yaw_error) > self.orthogonal_yaw_tolerance
        ):
            cmd.angular.z = self.angular_command(final_yaw_error)
            self.publish_status(
                'orthogonal_yaw_align',
                plan,
                cmd,
                distance=distance,
                yaw_error=final_yaw_error,
                body_x=body_x,
                body_y=body_y,
                action=self.movement_name(cmd),
                reason=(
                    'yaw误差 %.1fdeg 超过转向启动阈值 %.1fdeg'
                    % (
                        math.degrees(final_yaw_error),
                        math.degrees(self.orthogonal_yaw_tolerance),
                    )
                ),
            )
            return cmd

        if abs(body_x) > self.linear_tolerance:
            cmd.linear.x = self.axis_command(
                body_x,
                self.linear_gain,
                self.min_linear_speed,
                self.max_linear_speed,
            )
            self.publish_status(
                'orthogonal_forward_approach',
                plan,
                cmd,
                distance=distance,
                yaw_error=final_yaw_error,
                body_x=body_x,
                body_y=body_y,
                action=self.movement_name(cmd),
                reason=(
                    'body_y %.2fm 未超过横向阈值 %.2fm，yaw %.1fdeg 未超过转向阈值 %.1fdeg，优先前后接近'
                    % (
                        body_y,
                        self.orthogonal_lateral_tolerance,
                        math.degrees(final_yaw_error),
                        math.degrees(self.orthogonal_yaw_tolerance),
                    )
                ),
            )
            return cmd

        if self.enable_yaw_control and abs(final_yaw_error) > self.yaw_tolerance:
            cmd.angular.z = self.angular_command(final_yaw_error)
            self.publish_status(
                'orthogonal_final_yaw_align',
                plan,
                cmd,
                distance=distance,
                yaw_error=final_yaw_error,
                body_x=body_x,
                body_y=body_y,
                action=self.movement_name(cmd),
                reason=(
                    '已到停车点附近，做最后朝向修正 %.1fdeg'
                    % math.degrees(final_yaw_error)
                ),
            )
            return cmd

        self.handle_arrival(plan)
        self.publish_status(
            'arrived',
            plan,
            cmd,
            distance=distance,
            yaw_error=0.0,
            body_x=body_x,
            body_y=body_y,
            action='停止',
            reason='body_x、body_y、yaw 均进入到达阈值',
        )
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

        if self.drive_mode == 'orthogonal':
            return self.command_for_relative_orthogonal_plan(
                plan,
                goal_x,
                goal_y,
                distance_to_stop,
                yaw_error,
            )

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

    def command_for_relative_orthogonal_plan(
        self,
        plan: NavPlan,
        goal_x: float,
        goal_y: float,
        distance_to_stop: float,
        yaw_error: float,
    ) -> Twist:
        cmd = Twist()

        if abs(goal_y) > self.orthogonal_lateral_tolerance:
            cmd.linear.y = self.axis_command(
                goal_y,
                self.lateral_gain,
                self.min_lateral_speed,
                self.max_lateral_speed,
            )
            self.publish_status(
                'relative_orthogonal_lateral_align',
                plan,
                cmd,
                distance=distance_to_stop,
                yaw_error=yaw_error,
                body_x=goal_x,
                body_y=goal_y,
                action=self.movement_name(cmd),
                reason=(
                    'goal_y %.2fm 超过横向启动阈值 %.2fm'
                    % (goal_y, self.orthogonal_lateral_tolerance)
                ),
            )
            return cmd

        if self.enable_yaw_control and abs(yaw_error) > self.orthogonal_yaw_tolerance:
            cmd.angular.z = self.angular_command(yaw_error)
            self.publish_status(
                'relative_orthogonal_yaw_align',
                plan,
                cmd,
                distance=distance_to_stop,
                yaw_error=yaw_error,
                body_x=goal_x,
                body_y=goal_y,
                action=self.movement_name(cmd),
                reason=(
                    'yaw误差 %.1fdeg 超过转向启动阈值 %.1fdeg'
                    % (
                        math.degrees(yaw_error),
                        math.degrees(self.orthogonal_yaw_tolerance),
                    )
                ),
            )
            return cmd

        if abs(goal_x) > self.linear_tolerance:
            cmd.linear.x = self.axis_command(
                goal_x,
                self.linear_gain,
                self.min_linear_speed,
                self.max_linear_speed,
            )
            self.publish_status(
                'relative_orthogonal_forward_approach',
                plan,
                cmd,
                distance=distance_to_stop,
                yaw_error=yaw_error,
                body_x=goal_x,
                body_y=goal_y,
                action=self.movement_name(cmd),
                reason=(
                    'goal_y %.2fm 未超过横向阈值 %.2fm，yaw %.1fdeg 未超过转向阈值 %.1fdeg，优先前后接近'
                    % (
                        goal_y,
                        self.orthogonal_lateral_tolerance,
                        math.degrees(yaw_error),
                        math.degrees(self.orthogonal_yaw_tolerance),
                    )
                ),
            )
            return cmd

        if self.enable_yaw_control and abs(yaw_error) > self.yaw_tolerance:
            cmd.angular.z = self.angular_command(yaw_error)
            self.publish_status(
                'relative_orthogonal_final_yaw_align',
                plan,
                cmd,
                distance=distance_to_stop,
                yaw_error=yaw_error,
                body_x=goal_x,
                body_y=goal_y,
                action=self.movement_name(cmd),
                reason='已到停车点附近，做最后朝向修正 %.1fdeg'
                % math.degrees(yaw_error),
            )
            return cmd

        self.handle_arrival(plan)
        self.publish_status(
            'arrived',
            plan,
            cmd,
            distance=distance_to_stop,
            yaw_error=0.0,
            body_x=goal_x,
            body_y=goal_y,
            action='停止',
            reason='goal_x、goal_y、yaw 均进入到达阈值',
        )
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
        body_x: float | None = None,
        body_y: float | None = None,
        action: str | None = None,
        reason: str | None = None,
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
        if body_x is not None:
            parts.append('body_x=%.3f' % body_x)
        if body_y is not None:
            parts.append('body_y=%.3f' % body_y)
        if action is not None:
            parts.append('action=%s' % action)
        if reason is not None:
            parts.append('reason=%s' % reason)
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
        self.maybe_publish_debug_status(
            state,
            plan,
            cmd,
            distance=distance,
            yaw_error=yaw_error,
            body_x=body_x,
            body_y=body_y,
            action=action,
            reason=reason,
        )

    def maybe_publish_debug_status(
        self,
        state: str,
        plan: NavPlan | None,
        cmd: Twist | None,
        *,
        distance: float | None,
        yaw_error: float | None,
        body_x: float | None,
        body_y: float | None,
        action: str | None,
        reason: str | None,
    ) -> None:
        if not self.nav_debug_log:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_debug_log_time).nanoseconds / 1e9
        debug_key = '%s:%s' % (state, action or '')
        if (
            self.last_debug_state == debug_key
            and self.nav_debug_log_period > 0.0
            and elapsed < self.nav_debug_log_period
        ):
            return

        self.last_debug_log_time = now
        self.last_debug_state = debug_key
        text = self.format_debug_status(
            state,
            plan,
            cmd,
            distance=distance,
            yaw_error=yaw_error,
            body_x=body_x,
            body_y=body_y,
            action=action,
            reason=reason,
        )
        msg = String()
        msg.data = text
        self.debug_status_pub.publish(msg)
        self.get_logger().info(text)

    def format_debug_status(
        self,
        state: str,
        plan: NavPlan | None,
        cmd: Twist | None,
        *,
        distance: float | None,
        yaw_error: float | None,
        body_x: float | None,
        body_y: float | None,
        action: str | None,
        reason: str | None,
    ) -> str:
        if plan is not None and self.pose is not None and (
            body_x is None or body_y is None
        ):
            dx = plan.goal_x - self.pose.x
            dy = plan.goal_y - self.pose.y
            body_x, body_y = self.fixed_error_to_body(dx, dy)

        if distance is None and plan is not None and self.pose is not None:
            distance = math.hypot(plan.goal_x - self.pose.x, plan.goal_y - self.pose.y)

        action = action or (self.movement_name(cmd) if cmd is not None else '等待')
        reason = reason or self.default_reason(state)
        gait_motion = self.gait_motion_zh()
        command_source = self.gait_status_value('command_source') or '未收到'
        target_age = self.target_age_seconds()

        target_text = '无目标'
        if plan is not None:
            target_text = (
                '来源=%s 原始=(%.2f,%.2f) %s箱子=(%.2f,%.2f) 停车点=(%.2f,%.2f)'
                % (
                    plan.source_frame_id,
                    plan.raw_target_x,
                    plan.raw_target_y,
                    plan.frame_id,
                    plan.target_x,
                    plan.target_y,
                    plan.goal_x,
                    plan.goal_y,
                )
            )

        box_body_x, box_body_y = self.box_body_position(plan)
        box_position_text = self.describe_box_position(box_body_x, box_body_y)
        correction_text = self.describe_correction(
            action,
            body_x,
            body_y,
            yaw_error,
        )

        error_text = (
            '停车点body_x=%s body_y=%s 距离=%s yaw=%s 目标年龄=%s'
            % (
                self.format_meters(body_x),
                self.format_meters(body_y),
                self.format_meters(distance),
                self.format_degrees(yaw_error),
                self.format_seconds(target_age),
            )
        )

        cmd_text = 'x=0.000 y=0.000 yaw=0.000'
        if cmd is not None:
            cmd_text = 'x=%.3f y=%.3f yaw=%.3f' % (
                cmd.linear.x,
                cmd.linear.y,
                cmd.angular.z,
            )

        return (
            '[导航] 状态=%s 模式=%s/%s 发速度=%s 步态=%s | '
            '[目标] %s | [方位] %s | [误差] %s | [修正] %s | [决策] 想要=%s 原因=%s | '
            '[输出] cmd_vel %s 底层来源=%s'
            % (
                self.state_name_zh(state),
                self.control_mode,
                self.drive_mode,
                '是' if self.publish_cmd_vel else '否',
                gait_motion,
                target_text,
                box_position_text,
                error_text,
                correction_text,
                action,
                reason,
                cmd_text,
                command_source,
            )
        )

    def default_reason(self, state: str) -> str:
        if state == 'waiting_for_target':
            return '还没有收到箱子目标'
        if state in ('waiting_for_odom', 'waiting_for_odom_projection'):
            return '还没有收到 odom'
        if state == 'preview_only':
            return 'publish_cmd_vel=false，只预览不发速度'
        if state == 'target_timeout_stop':
            return '目标数据超时，安全停车'
        if state in ('arrived', 'arrived_hold'):
            return '已经到达或保持到达状态'
        return '查看误差和输出判断'

    def movement_name(self, cmd: Twist | None) -> str:
        if cmd is None:
            return '等待'
        if abs(cmd.linear.y) > 1e-6:
            return '左移' if cmd.linear.y > 0.0 else '右移'
        if abs(cmd.angular.z) > 1e-6:
            return '左转' if cmd.angular.z > 0.0 else '右转'
        if abs(cmd.linear.x) > 1e-6:
            return '前进' if cmd.linear.x > 0.0 else '后退'
        return '停止'

    def box_body_position(
        self, plan: NavPlan | None
    ) -> tuple[float | None, float | None]:
        if plan is None:
            return None, None
        if self.pose is not None and plan.frame_id == self.fixed_frame:
            dx = plan.target_x - self.pose.x
            dy = plan.target_y - self.pose.y
            return self.fixed_error_to_body(dx, dy)
        if plan.source_frame_id != self.fixed_frame:
            return plan.raw_target_x + self.sensor_forward_offset, plan.raw_target_y
        return plan.target_x, plan.target_y

    def describe_box_position(
        self, body_x: float | None, body_y: float | None
    ) -> str:
        if body_x is None or body_y is None:
            return '箱子方位未知'

        if body_x >= 0.0:
            forward_text = '前方%.2fm' % abs(body_x)
        else:
            forward_text = '后方%.2fm' % abs(body_x)

        if abs(body_y) < 0.05:
            lateral_text = '横向基本居中'
        elif body_y > 0.0:
            lateral_text = '左侧%.2fm' % body_y
        else:
            lateral_text = '右侧%.2fm' % abs(body_y)

        return (
            '箱子在机身%s、%s；车体系坐标 x前+=%.2f y左+=%.2f'
            % (forward_text, lateral_text, body_x, body_y)
        )

    def describe_correction(
        self,
        action: str,
        body_x: float | None,
        body_y: float | None,
        yaw_error: float | None,
    ) -> str:
        if action == '左移':
            return '机器向左横移，修正停车点横向误差 body_y=%s' % self.format_meters(body_y)
        if action == '右移':
            return '机器向右横移，修正停车点横向误差 body_y=%s' % self.format_meters(body_y)
        if action == '左转':
            return '机器向左原地转，修正朝向误差 yaw=%s' % self.format_degrees(yaw_error)
        if action == '右转':
            return '机器向右原地转，修正朝向误差 yaw=%s' % self.format_degrees(yaw_error)
        if action == '前进':
            return '机器向前走，修正到停车点的前后距离 body_x=%s' % self.format_meters(body_x)
        if action == '后退':
            return '机器向后退，修正到停车点的前后距离 body_x=%s' % self.format_meters(body_x)
        if action == '停止':
            return '机器停止，当前认为已经到达或需要安全停车'
        if action == '预览':
            return '只预览不发速度；根据误差判断下一步动作'
        return '机器等待；当前没有可执行修正'

    def state_name_zh(self, state: str) -> str:
        names = {
            'waiting_for_target': '等待目标',
            'waiting_for_odom': '等待 odom',
            'waiting_for_odom_projection': '等待 odom 投影',
            'preview_only': '仅预览',
            'target_timeout_stop': '目标超时停车',
            'arrived_hold': '到达保持',
            'arrived': '到达',
            'rotate_to_goal': '转向目标',
            'drive_forward': '前进',
            'align_at_goal': '终点对齐',
            'holonomic_drive': '全向接近',
            'orthogonal_lateral_align': '正交横向对齐',
            'orthogonal_yaw_align': '正交原地转向',
            'orthogonal_forward_approach': '正交前后接近',
            'orthogonal_final_yaw_align': '正交终点转向',
            'relative_orthogonal_lateral_align': '相对正交横向对齐',
            'relative_orthogonal_yaw_align': '相对正交原地转向',
            'relative_orthogonal_forward_approach': '相对正交前后接近',
            'relative_orthogonal_final_yaw_align': '相对正交终点转向',
        }
        return names.get(state, state)

    def gait_motion_zh(self) -> str:
        motion = self.gait_status_value('motion')
        names = {
            'stand_up': '站起',
            'stand_hold': '站立保持',
            'walk': '行走',
        }
        if motion is None:
            return '未收到'
        return names.get(motion, motion)

    def gait_status_value(self, key: str) -> str | None:
        if not self.latest_gait_status:
            return None
        marker = key + '='
        start = self.latest_gait_status.find(marker)
        if start < 0:
            return None
        start += len(marker)
        end = self.latest_gait_status.find(',', start)
        if end < 0:
            end = len(self.latest_gait_status)
        return self.latest_gait_status[start:end].strip()

    def format_meters(self, value: float | None) -> str:
        if value is None:
            return '--'
        return '%.2fm' % value

    def format_degrees(self, value: float | None) -> str:
        if value is None:
            return '--'
        return '%.1fdeg' % math.degrees(value)

    def format_seconds(self, value: float | None) -> str:
        if value is None:
            return '--'
        return '%.2fs' % value

    def log_plan(self, plan: NavPlan) -> None:
        self.get_logger().info(
            '[规划] 起点(%.2f, %.2f) -> 停车点(%.2f, %.2f) -> 箱子(%.2f, %.2f), 朝向 %.1fdeg, 原始目标%s(%.2f, %.2f)'
            % (
                plan.start_x,
                plan.start_y,
                plan.goal_x,
                plan.goal_y,
                plan.target_x,
                plan.target_y,
                math.degrees(plan.goal_yaw),
                plan.source_frame_id,
                plan.raw_target_x,
                plan.raw_target_y,
            )
        )

    def log_command(self, plan: NavPlan, cmd: Twist) -> None:
        if self.nav_debug_log:
            return
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
