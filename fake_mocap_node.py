#!/usr/bin/env python3
"""
fake_mocap_node.py — publishes static fake MoCap poses for development.

Publishes the same topics as the real VRPN node:
  /vrpn_mocap/{name}/pose  for each rigid_body in fake_mocap_poses.json

Edit fake_mocap_poses.json to add/remove/reposition obstacles.
Poses are auto-reloaded every 5s — no restart needed.

Usage:
  make fake-mocap
"""
import json
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
POSES_FILE  = os.path.join(PROJECT_DIR, 'fake_mocap_poses.json')
NAMESPACE   = '/vrpn_mocap'


def load_config():
    with open(POSES_FILE, encoding='utf-8') as f:
        return json.load(f)


class FakeMocapNode(Node):
    def __init__(self):
        super().__init__('fake_mocap_node')
        self._cfg  = load_config()
        self._pubs = {}
        self._build_publishers()
        hz = float(self._cfg.get('publish_rate_hz', 100.0))
        self.create_timer(1.0 / hz, self._publish)
        self.create_timer(5.0, self._reload)
        self.get_logger().info(
            f'Fake MoCap ready — publishing {len(self._pubs)} rigid bodies '
            f'at {hz:.0f} Hz from {POSES_FILE}')
        self.get_logger().info(
            'Edit fake_mocap_poses.json to change positions (auto-reloaded every 5s)')

    def _build_publishers(self):
        bodies = self._cfg.get('rigid_bodies', {})
        for name in bodies:
            topic = f'{NAMESPACE}/{name}/pose'
            if name not in self._pubs:
                self._pubs[name] = self.create_publisher(
                    PoseStamped, topic, 10)
                self.get_logger().info(f'  {topic}')

    def _reload(self):
        try:
            self._cfg = load_config()
            # add any new rigid bodies that appeared in the file
            self._build_publishers()
        except Exception as e:
            self.get_logger().warn(f'Reload failed: {e}')

    def _publish(self):
        stamp  = self.get_clock().now().to_msg()
        bodies = self._cfg.get('rigid_bodies', {})
        for name, body in bodies.items():
            if name not in self._pubs:
                continue
            msg = PoseStamped()
            msg.header.stamp    = stamp
            msg.header.frame_id = 'world'
            p = body['position']
            q = body['orientation']
            msg.pose.position.x    = float(p[0])
            msg.pose.position.y    = float(p[1])
            msg.pose.position.z    = float(p[2])
            msg.pose.orientation.x = float(q[0])
            msg.pose.orientation.y = float(q[1])
            msg.pose.orientation.z = float(q[2])
            msg.pose.orientation.w = float(q[3])
            self._pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMocapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
