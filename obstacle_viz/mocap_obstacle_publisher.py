#!/usr/bin/env python3
"""Publish live MoCap obstacles as RViz markers in the ``map`` frame."""
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from mocap_obstacles import (  # noqa: E402
    MOCAP_POSE_TOPICS,
    MOCAP_STATIC_NAMES,
    MOCAP_OBSTACLE_SIZES,
    MOCAP_OBSTACLE_COLORS,
    mocap_position_to_map,
)


def _hex_rgb(hex_color: str):
    h = hex_color.lstrip('#')
    return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


class MocapObstaclePublisher(Node):
    def __init__(self):
        super().__init__('mocap_obstacle_publisher')
        self._poses = {}          # index 1..3 → (map_x, map_y, map_z)
        self._logged = set()
        self._static_tf = StaticTransformBroadcaster(self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._groundplane = None
        self._publish_map_tf()  # identity fallback until groundplane arrives
        self.create_subscription(
            PoseStamped, '/vrpn_mocap/groundplane/pose',
            self._groundplane_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(MarkerArray, '/mocap_obstacles', 10)
        for idx, topic in enumerate(MOCAP_POSE_TOPICS, start=1):
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, i=idx: self._pose_cb(msg, i),
                qos_profile_sensor_data)
        self.create_timer(0.05, self._publish_markers)
        self.get_logger().info(
            'MoCap RViz publisher ready — /mocap_obstacles (fixed frame: map)')

    def _publish_map_tf(self):
        """Broadcast identity world→map as a latched fallback."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.rotation.w = 1.0
        self._static_tf.sendTransform(t)

    def _groundplane_cb(self, msg):
        """Rebroadcast world→map as inverse of groundplane pose."""
        p = msg.pose.position
        q = msg.pose.orientation
        # Inverse of groundplane pose = conjugate quaternion + rotated translation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        # Conjugate (inverse rotation)
        inv_qx, inv_qy, inv_qz, inv_qw = -qx, -qy, -qz, qw
        # Inverse translation: rotate -p by inverse quaternion
        # v' = q_inv * v * q  (pure quaternion sandwich)
        px, py, pz = p.x, p.y, p.z
        # Rotate vector (px,py,pz) by conjugate quaternion
        tx = (1-2*(inv_qy**2+inv_qz**2))*(-px) + 2*(inv_qx*inv_qy-inv_qz*inv_qw)*(-py) + 2*(inv_qx*inv_qz+inv_qy*inv_qw)*(-pz)
        ty = 2*(inv_qx*inv_qy+inv_qz*inv_qw)*(-px) + (1-2*(inv_qx**2+inv_qz**2))*(-py) + 2*(inv_qy*inv_qz-inv_qx*inv_qw)*(-pz)
        tz = 2*(inv_qx*inv_qz-inv_qy*inv_qw)*(-px) + 2*(inv_qy*inv_qz+inv_qx*inv_qw)*(-py) + (1-2*(inv_qx**2+inv_qy**2))*(-pz)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.x = inv_qx
        t.transform.rotation.y = inv_qy
        t.transform.rotation.z = inv_qz
        t.transform.rotation.w = inv_qw
        self._tf_broadcaster.sendTransform(t)

    def _pose_cb(self, msg, index: int):
        p = msg.pose.position
        map_x, map_y, map_z = mocap_position_to_map(
            float(p.x), float(p.y), float(p.z))
        self._poses[index] = (map_x, map_y, map_z)
        if index not in self._logged:
            self._logged.add(index)
            self.get_logger().info(
                f'obstacle{index}: mocap ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) '
                f'→ map ({map_x:.2f}, {map_y:.2f}, {map_z:.2f})')

    def _publish_markers(self):
        if not self._poses:
            return
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        for idx in sorted(self._poses.keys()):
            map_x, map_y, map_z = self._poses[idx]
            w, h = MOCAP_OBSTACLE_SIZES[idx - 1] if idx - 1 < len(MOCAP_OBSTACLE_SIZES) else MOCAP_OBSTACLE_SIZES[-1]
            face_hex = MOCAP_OBSTACLE_COLORS[idx][0]
            r, g, b = _hex_rgb(face_hex)

            cube = Marker()
            cube.header.frame_id = 'world'
            cube.header.stamp = stamp
            cube.ns = 'mocap_obstacles'
            cube.id = idx
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = map_x
            cube.pose.position.y = map_y
            cube.pose.position.z = max(0.0, map_z) + h / 2.0
            cube.pose.orientation.w = 1.0
            cube.scale.x = w
            cube.scale.y = h
            cube.scale.z = h
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = r, g, b, 0.85
            array.markers.append(cube)

            label = Marker()
            label.header.frame_id = 'world'
            label.header.stamp = stamp
            label.ns = 'mocap_labels'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = map_x
            label.pose.position.y = map_y
            label.pose.position.z = map_z + h + 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = (
                MOCAP_STATIC_NAMES[idx - 1]
                if idx - 1 < len(MOCAP_STATIC_NAMES) else f'obstacle{idx}')
            array.markers.append(label)

        self.pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = MocapObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
