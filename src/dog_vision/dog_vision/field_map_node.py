#!/usr/bin/env python3
"""Publish an ideal task-field map and manual mission targets."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


def package_config_path() -> str:
    share = Path(get_package_share_directory('dog_vision'))
    return str(share / 'config' / 'field_map.yaml')


def yaw_to_quaternion(pose, yaw: float) -> None:
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)


def point(x: float, y: float, z: float = 0.0) -> Point:
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


class FieldMapNode(Node):
    def __init__(self) -> None:
        super().__init__('field_map_node')

        self.declare_parameter('map_config_file', package_config_path())
        self.declare_parameter('frame_id_override', '')
        self.declare_parameter('marker_topic', '/navigation/field_map_markers')
        self.declare_parameter('target_pose_topic', '/navigation/field_target_pose')
        self.declare_parameter('target_box_pose_topic', '/navigation/field_box_pose')
        self.declare_parameter('target_id_topic', '/vision/target_id')
        self.declare_parameter('return_zone_id_topic', '/navigation/return_zone_id')
        self.declare_parameter('nav_status_topic', '/navigation/nav_status')
        self.declare_parameter('mission_command_topic', '/navigation/field_mission_command')
        self.declare_parameter('status_topic', '/navigation/field_map_status')
        self.declare_parameter('target_row', 0)
        self.declare_parameter('target_col', 1)
        self.declare_parameter('target_return_zone', 0)
        self.declare_parameter('mission_mode', 'box_only')
        self.declare_parameter('publish_target_pose', True)
        self.declare_parameter('publish_period', 0.5)

        self.map_config_file = str(self.get_parameter('map_config_file').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.target_pose_topic = str(self.get_parameter('target_pose_topic').value)
        self.target_box_pose_topic = str(
            self.get_parameter('target_box_pose_topic').value
        )
        self.target_id_topic = str(self.get_parameter('target_id_topic').value)
        self.return_zone_id_topic = str(
            self.get_parameter('return_zone_id_topic').value
        )
        self.nav_status_topic = str(self.get_parameter('nav_status_topic').value)
        self.mission_command_topic = str(
            self.get_parameter('mission_command_topic').value
        )
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.target_row = int(self.get_parameter('target_row').value)
        self.target_col = int(self.get_parameter('target_col').value)
        self.target_return_zone = int(
            self.get_parameter('target_return_zone').value
        )
        self.mission_mode = str(self.get_parameter('mission_mode').value).lower()
        if self.mission_mode not in ('box_only', 'box_then_return', 'return_only'):
            self.get_logger().warning(
                "Unknown mission_mode '%s', using box_only." % self.mission_mode
            )
            self.mission_mode = 'box_only'
        self.mission_stage = 'return' if self.mission_mode == 'return_only' else 'box'
        self.publish_target_pose_enabled = bool(
            self.get_parameter('publish_target_pose').value
        )
        self.publish_period = float(self.get_parameter('publish_period').value)

        self.config = self.load_config(self.map_config_file)
        frame_override = str(self.get_parameter('frame_id_override').value)
        self.frame_id = frame_override or str(self.config.get('frame_id', 'odom'))
        height_cfg = self.config.get('height', {})
        self.odom_to_ground = float(height_cfg.get('odom_to_ground', 0.0))
        self.target_pose_z = float(
            height_cfg.get('target_pose_z', -self.odom_to_ground)
        )
        marker_cfg = self.config.get('markers', {})
        self.ground_z = float(marker_cfg.get('ground_z', -self.odom_to_ground))
        self.text_height = float(marker_cfg.get('text_height', 0.16))

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.target_pose_pub = self.create_publisher(
            PoseStamped, self.target_pose_topic, 10
        )
        self.target_box_pose_pub = self.create_publisher(
            PoseStamped, self.target_box_pose_topic, 10
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.target_id_sub = self.create_subscription(
            Int32MultiArray, self.target_id_topic, self.target_id_callback, 10
        )
        self.return_zone_id_sub = self.create_subscription(
            Int32,
            self.return_zone_id_topic,
            self.return_zone_id_callback,
            10,
        )
        self.nav_status_sub = self.create_subscription(
            String,
            self.nav_status_topic,
            self.nav_status_callback,
            10,
        )
        self.mission_command_sub = self.create_subscription(
            String,
            self.mission_command_topic,
            self.mission_command_callback,
            10,
        )

        period = max(0.1, self.publish_period)
        self.timer = self.create_timer(period, self.publish_outputs)
        self.get_logger().info(
            'Field map started: config=%s frame=%s target=(%d,%d) return=%d mode=%s stage=%s ground_z=%.3f'
            % (
                self.map_config_file,
                self.frame_id,
                self.target_row,
                self.target_col,
                self.target_return_zone,
                self.mission_mode,
                self.mission_stage,
                self.ground_z,
            )
        )

    @staticmethod
    def load_config(path: str) -> dict[str, Any]:
        with Path(path).open('r', encoding='utf-8') as file_obj:
            data = yaml.safe_load(file_obj) or {}
        if not isinstance(data, dict):
            raise RuntimeError('field map config must be a YAML dictionary')
        return data

    def target_id_callback(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != 2:
            self.get_logger().warning('Ignoring target id: expected [row, col].')
            return
        row = int(msg.data[0])
        col = int(msg.data[1])
        if not self.valid_storage_index(row, col):
            self.get_logger().warning('Ignoring invalid target id: [%d, %d].' % (row, col))
            return
        if row != self.target_row or col != self.target_col:
            self.target_row = row
            self.target_col = col
            self.mission_stage = 'box'
            self.get_logger().info('Manual target -> row=%d col=%d' % (row, col))

    def return_zone_id_callback(self, msg: Int32) -> None:
        zone_index = int(msg.data)
        if not self.valid_return_zone_index(zone_index):
            self.get_logger().warning(
                'Ignoring invalid return zone id: %d.' % zone_index
            )
            return
        if zone_index != self.target_return_zone:
            self.target_return_zone = zone_index
            self.get_logger().info('Manual return zone -> id=%d' % zone_index)

    def nav_status_callback(self, msg: String) -> None:
        if self.mission_mode != 'box_then_return' or self.mission_stage != 'box':
            return
        if self.nav_state(msg.data) not in ('arrived', 'arrived_hold'):
            return
        self.mission_stage = 'return'
        self.get_logger().info(
            'Box approach reached. Switching target to return zone %d.'
            % self.target_return_zone
        )

    def mission_command_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in ('box_only', 'box_then_return', 'return_only'):
            self.mission_mode = command
            self.mission_stage = 'return' if command == 'return_only' else 'box'
            self.get_logger().info(
                'Mission mode -> %s stage=%s'
                % (self.mission_mode, self.mission_stage)
            )
            return
        if command in ('start', 'reset', 'box'):
            self.mission_stage = 'box'
            self.get_logger().info('Mission stage -> box')
            return
        if command in ('return', 'return_zone'):
            self.mission_stage = 'return'
            self.get_logger().info('Mission stage -> return')
            return
        self.get_logger().warning('Ignoring mission command: %s' % msg.data)

    @staticmethod
    def nav_state(status_text: str) -> str:
        for part in status_text.split(','):
            key, _, value = part.strip().partition('=')
            if key == 'state':
                return value.strip()
        return ''

    def valid_storage_index(self, row: int, col: int) -> bool:
        grid = self.config['storage_grid']
        return 0 <= row < int(grid['rows']) and 0 <= col < int(grid['cols'])

    def valid_return_zone_index(self, zone_index: int) -> bool:
        zones = self.config['return_zones'].get('zones', [])
        return 0 <= zone_index < len(zones)

    def storage_center(self, row: int, col: int) -> tuple[float, float]:
        grid = self.config['storage_grid']
        x = float(grid['row0_center_x']) + row * float(grid['center_spacing_x'])
        col0_y = float(grid['col0_center_y'])
        spacing_y = float(grid['center_spacing_y'])
        positive_y_left = bool(grid.get('sort_left_to_right_positive_y', True))
        y = col0_y - col * spacing_y if positive_y_left else col0_y + col * spacing_y
        return x, y

    def return_zone_center(self, zone_index: int) -> tuple[float, float]:
        cfg = self.config['return_zones']
        center_x = float(cfg['center_x'])
        col0_y = float(cfg['col0_center_y'])
        spacing_y = float(cfg['center_spacing_y'])
        return center_x, col0_y - zone_index * spacing_y

    def selected_box_goal(self) -> tuple[float, float, float, float, float]:
        box_x, box_y = self.storage_center(self.target_row, self.target_col)
        approach = self.config['storage_grid'].get('approach', {})
        goal_x, goal_y, goal_yaw = self.approach_pose_for(box_x, box_y, approach)
        return box_x, box_y, goal_x, goal_y, goal_yaw

    def selected_return_goal(self) -> tuple[float, float, float, float, float]:
        return_x, return_y = self.return_zone_center(self.target_return_zone)
        approach = self.config['return_zones'].get('approach', {})
        goal_x, goal_y, goal_yaw = self.approach_pose_for(
            return_x,
            return_y,
            approach,
        )
        return return_x, return_y, goal_x, goal_y, goal_yaw

    def active_goal(self) -> tuple[str, float, float, float, float, float]:
        if self.mission_mode == 'return_only' or self.mission_stage == 'return':
            center_x, center_y, goal_x, goal_y, goal_yaw = self.selected_return_goal()
            return 'return', center_x, center_y, goal_x, goal_y, goal_yaw
        center_x, center_y, goal_x, goal_y, goal_yaw = self.selected_box_goal()
        return 'box', center_x, center_y, goal_x, goal_y, goal_yaw

    def approach_pose_for(
        self,
        center_x: float,
        center_y: float,
        approach: dict[str, Any],
    ) -> tuple[float, float, float]:
        side = str(approach.get('side', 'negative_x'))
        standoff = float(approach.get('standoff', 0.60))
        dx, dy = self.side_vector(side)
        goal_x = center_x + dx * standoff
        goal_y = center_y + dy * standoff
        yaw = math.atan2(center_y - goal_y, center_x - goal_x)
        return goal_x, goal_y, yaw

    @staticmethod
    def side_vector(side: str) -> tuple[float, float]:
        side = side.lower()
        if side == 'positive_x':
            return 1.0, 0.0
        if side == 'positive_y':
            return 0.0, 1.0
        if side == 'negative_y':
            return 0.0, -1.0
        return -1.0, 0.0

    def publish_outputs(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self.delete_all_marker(now))
        markers.markers.extend(self.field_markers(now))
        markers.markers.extend(self.storage_markers(now))
        markers.markers.extend(self.return_zone_markers(now))
        markers.markers.extend(self.speed_bump_markers(now))
        markers.markers.extend(self.selected_target_markers(now))
        self.marker_pub.publish(markers)

        active_kind, center_x, center_y, goal_x, goal_y, goal_yaw = self.active_goal()
        box_x, box_y, _, _, box_goal_yaw = self.selected_box_goal()
        if self.publish_target_pose_enabled:
            self.target_pose_pub.publish(
                self.pose_msg(now, goal_x, goal_y, goal_yaw)
            )
            self.target_box_pose_pub.publish(
                self.pose_msg(now, box_x, box_y, box_goal_yaw)
            )

        status = String()
        status.data = (
            'frame=%s, ground_z=%.3f, mode=%s, stage=%s, active=%s, box=(row=%d,col=%d center=%.3f,%.3f), return=%d center=(%.3f,%.3f), approach=(%.3f,%.3f,%.1fdeg)'
            % (
                self.frame_id,
                self.ground_z,
                self.mission_mode,
                self.mission_stage,
                active_kind,
                self.target_row,
                self.target_col,
                box_x,
                box_y,
                self.target_return_zone,
                center_x,
                center_y,
                goal_x,
                goal_y,
                math.degrees(goal_yaw),
            )
        )
        self.status_pub.publish(status)

    def delete_all_marker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'field_map'
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def field_markers(self, stamp) -> list[Marker]:
        field = self.config['field']
        start_cfg = self.config.get('start_zone', {})
        near_x = float(field['near_wall_x'])
        far_x = float(field['far_wall_x'])
        right_y = float(field['right_wall_y'])
        left_y = float(field['left_wall_y'])
        start_center = start_cfg.get('center', [0.0, 0.0])
        start_size = start_cfg.get('size', [1.0, 1.0])

        boundary = self.base_marker(stamp, 1, 'field_boundary', Marker.LINE_STRIP)
        boundary.scale.x = 0.035
        self.set_color(boundary, [1.0, 0.45, 0.0, 0.95])
        boundary.points = [
            point(near_x, right_y, self.map_z(0.03)),
            point(far_x, right_y, self.map_z(0.03)),
            point(far_x, left_y, self.map_z(0.03)),
            point(near_x, left_y, self.map_z(0.03)),
            point(near_x, right_y, self.map_z(0.03)),
        ]

        start = self.box_marker(
            stamp,
            2,
            'start_zone',
            float(start_center[0]),
            float(start_center[1]),
            self.map_z(0.015),
            float(start_size[0]),
            float(start_size[1]),
            0.03,
            [1.0, 1.0, 1.0, 0.20],
        )
        label = self.text_marker(
            stamp,
            3,
            'field_labels',
            float(start_center[0]),
            float(start_center[1]),
            self.map_z(0.22),
            'START',
        )
        return [boundary, start, label]

    def storage_markers(self, stamp) -> list[Marker]:
        markers: list[Marker] = []
        grid = self.config['storage_grid']
        box_size = float(grid['box_size'])
        zone_size = grid.get('zone_size', [box_size, box_size])
        rows = int(grid['rows'])
        cols = int(grid['cols'])
        marker_id = 100
        for row in range(rows):
            for col in range(cols):
                center_x, center_y = self.storage_center(row, col)
                is_target = row == self.target_row and col == self.target_col
                zone_color = [1.0, 0.95, 0.15, 0.18]
                box_color = [0.75, 0.50, 0.25, 0.45]
                if is_target:
                    zone_color = [1.0, 0.05, 0.05, 0.35]
                    box_color = [1.0, 0.05, 0.05, 0.55]
                markers.append(
                    self.box_marker(
                        stamp,
                        marker_id,
                        'storage_zones',
                        center_x,
                        center_y,
                        self.map_z(0.01),
                        float(zone_size[0]),
                        float(zone_size[1]),
                        0.02,
                        zone_color,
                    )
                )
                marker_id += 1
                markers.append(
                    self.box_marker(
                        stamp,
                        marker_id,
                        'storage_boxes',
                        center_x,
                        center_y,
                        self.map_z(box_size * 0.5),
                        box_size,
                        box_size,
                        box_size,
                        box_color,
                    )
                )
                marker_id += 1
                markers.append(
                    self.text_marker(
                        stamp,
                        marker_id,
                        'storage_labels',
                        center_x,
                        center_y,
                        self.map_z(box_size + 0.12),
                        'B%d%d' % (row, col),
                    )
                )
                marker_id += 1
        return markers

    def return_zone_markers(self, stamp) -> list[Marker]:
        markers: list[Marker] = []
        cfg = self.config['return_zones']
        size_x, size_y = cfg.get('zone_size', [0.40, 0.40])
        marker_id = 300
        for index, zone in enumerate(cfg.get('zones', [])):
            center_x, center_y = self.return_zone_center(index)
            color = zone.get('color', [0.5, 0.5, 0.5, 0.5])
            if index == self.target_return_zone:
                color = [float(color[0]), float(color[1]), float(color[2]), 0.85]
            label = '%s %d' % (zone.get('label', 'zone'), int(zone.get('id', index)))
            markers.append(
                self.box_marker(
                    stamp,
                    marker_id,
                    'return_zones',
                    center_x,
                    center_y,
                    self.map_z(0.015),
                    float(size_x),
                    float(size_y),
                    0.03,
                    color,
                )
            )
            marker_id += 1
            markers.append(
                self.text_marker(
                    stamp,
                    marker_id,
                    'return_labels',
                    center_x,
                    center_y,
                    self.map_z(0.18),
                    label,
                )
            )
            marker_id += 1
        return markers

    def speed_bump_markers(self, stamp) -> list[Marker]:
        bump = self.config['speed_bump']
        size_x, size_y, size_z = bump.get('size', [0.35, 4.0, 0.05])
        center_x = float(bump['center_x'])
        center_y = float(bump.get('center_y', 0.0))
        marker = self.box_marker(
            stamp,
            500,
            'speed_bump',
            center_x,
            center_y,
            self.map_z(float(size_z) * 0.5),
            float(size_x),
            float(size_y),
            float(size_z),
            [1.0, 0.85, 0.0, 0.45],
        )
        label = self.text_marker(
            stamp, 501, 'field_labels', center_x, center_y, self.map_z(0.16),
            'SPEED BUMP'
        )
        return [marker, label]

    def selected_target_markers(self, stamp) -> list[Marker]:
        active_kind, center_x, center_y, goal_x, goal_y, goal_yaw = self.active_goal()
        color = [0.1, 0.45, 1.0, 0.95]
        label_text = 'GOAL B%d%d' % (self.target_row, self.target_col)
        if active_kind == 'return':
            color = [0.0, 0.95, 0.25, 0.95]
            label_text = 'GOAL R%d' % self.target_return_zone

        arrow = self.base_marker(stamp, 700, 'selected_approach', Marker.ARROW)
        arrow.pose.position.x = goal_x
        arrow.pose.position.y = goal_y
        arrow.pose.position.z = self.map_z(0.08)
        yaw_to_quaternion(arrow.pose, goal_yaw)
        arrow.scale.x = 0.40
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        self.set_color(arrow, color)

        line = self.base_marker(stamp, 701, 'selected_approach', Marker.LINE_STRIP)
        line.scale.x = 0.035
        self.set_color(line, color)
        line.points = [
            point(goal_x, goal_y, self.map_z(0.05)),
            point(center_x, center_y, self.map_z(0.05)),
        ]

        label = self.text_marker(
            stamp,
            702,
            'selected_approach',
            goal_x,
            goal_y,
            self.map_z(0.24),
            label_text,
        )
        return [arrow, line, label]

    def pose_msg(self, stamp, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = self.target_pose_z
        yaw_to_quaternion(pose.pose, yaw)
        return pose

    def map_z(self, height_above_ground: float) -> float:
        return self.ground_z + float(height_above_ground)

    def base_marker(self, stamp, marker_id: int, ns: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def box_marker(
        self,
        stamp,
        marker_id: int,
        ns: str,
        x: float,
        y: float,
        z: float,
        sx: float,
        sy: float,
        sz: float,
        color: list[float],
    ) -> Marker:
        marker = self.base_marker(stamp, marker_id, ns, Marker.CUBE)
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.scale.x = float(sx)
        marker.scale.y = float(sy)
        marker.scale.z = float(sz)
        self.set_color(marker, color)
        return marker

    def text_marker(
        self,
        stamp,
        marker_id: int,
        ns: str,
        x: float,
        y: float,
        z: float,
        text: str,
    ) -> Marker:
        marker = self.base_marker(stamp, marker_id, ns, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.scale.z = self.text_height
        self.set_color(marker, [1.0, 1.0, 1.0, 0.95])
        marker.text = text
        return marker

    @staticmethod
    def set_color(marker: Marker, color: list[float]) -> None:
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3]) if len(color) > 3 else 1.0


def main() -> None:
    rclpy.init()
    node = FieldMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
