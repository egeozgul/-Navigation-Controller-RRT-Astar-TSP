#!/usr/bin/env python3
"""
pedestrian_sim_node.py — Pedestrian simulator for ROS2.

Publishes:
  /pedestrian_state   Float32MultiArray: [n, x0,y0,vx0,vy0, x1,y1,vx1,vy1, ...]
  /pedestrians        MarkerArray: cylinder markers for RViz

CLI args:
  --n_pedestrians N   initial count (default 8)
  --speed S           speed scale m/s (default 1.2)
  --hz H              update rate (default 25)
  --deploy            spawn within deployment viewport from mission_waypoints.json

Keyboard:
  h       add pedestrian
  j       remove pedestrian
  u       speed up
  t       speed down
  p       print status
  q       quit
"""
import argparse
import math
import sys
import os
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from APF_RRT_Astar import apply_stochastic_maneuver

# ── Constants ────────────────────────────────────────────────────────────────
MAX_POOL   = 60
DT         = 0.04   # 25 Hz step

# ── Pool builder ─────────────────────────────────────────────────────────────

def _build_pool(n, speed, bounds=None):
    """
    Build initial positions and velocities for n pedestrians.
    bounds: (x_min, x_max, y_min, y_max) for deployment, None for simulation.
    """
    rng = np.random.default_rng(42)

    if bounds is not None:
        xlo, xhi, ylo, yhi = bounds
        pos = np.column_stack([
            rng.uniform(xlo, xhi, MAX_POOL),
            rng.uniform(ylo, yhi, MAX_POOL),
        ])
    else:
        pos = rng.uniform(-45, 45, (MAX_POOL, 2))

    angles = rng.uniform(-np.pi, np.pi, MAX_POOL)
    if bounds is not None:
        sim_speed = speed  # deployment: direct m/s
    else:
        sim_speed = speed * (12.0 / 1.2)  # simulation units
    # All pedestrians start at the same speed magnitude
    spd = np.column_stack([
        np.cos(angles) * sim_speed,
        np.sin(angles) * sim_speed,
    ])
    return pos, spd


def _respawn(pos, spd, bounds):
    """Respawn pedestrians that have left the bounds."""
    rng = np.random.default_rng()
    if bounds is not None:
        xlo, xhi, ylo, yhi = bounds
        for i in range(len(pos)):
            x, y = pos[i]
            vx, vy = spd[i]
            # Bounce off walls
            if x < xlo or x > xhi:
                vx = -vx
                x = np.clip(x, xlo, xhi)
            if y < ylo or y > yhi:
                vy = -vy
                y = np.clip(y, ylo, yhi)
            pos[i] = [x, y]
            spd[i] = [vx, vy]

    else:
        for i in range(len(pos)):
            x, y = pos[i]
            if abs(x) > 50 or abs(y) > 50:
                edge = rng.integers(0, 4)
                if   edge == 0: pos[i] = [-48, rng.uniform(-48, 48)]
                elif edge == 1: pos[i] = [ 48, rng.uniform(-48, 48)]
                elif edge == 2: pos[i] = [rng.uniform(-48, 48), -48]
                else:           pos[i] = [rng.uniform(-48, 48),  48]
                angle = rng.uniform(-np.pi, np.pi)
                vmag2 = np.linalg.norm(spd[i])
                spd[i] = [np.cos(angle) * vmag2, np.sin(angle) * vmag2]
    return pos, spd


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class PedestrianSimNode(Node):
    def __init__(self, n_init, speed, hz, bounds):
        super().__init__('pedestrian_sim_node')
        self._lock   = threading.Lock()
        self._n      = min(max(1, n_init), MAX_POOL)
        self._speed  = speed
        self._bounds = bounds
        self._pos, self._spd = _build_pool(MAX_POOL, speed, bounds)

        qos = QoSProfile(depth=10)
        self._state_pub = self.create_publisher(
            Float32MultiArray, '/pedestrian_state', qos)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/pedestrians', qos)

        self.create_timer(1.0 / hz, self._step)
        self.get_logger().info(
            f'Pedestrian sim ready: n={n_init} speed={speed} hz={hz} '
            f'bounds={"deployment" if bounds else "simulation"}')

    # ── Controls ─────────────────────────────────────────────────────────────
    def add_ped(self):
        with self._lock:
            if self._n < MAX_POOL:
                self._n += 1
            n = self._n
        print(f'  Pedestrians: {n}')

    def rem_ped(self):
        with self._lock:
            if self._n > 1:
                self._n -= 1
            n = self._n
        print(f'  Pedestrians: {n}')

    def speed_up(self):
        with self._lock:
            self._speed = min(self._speed * 1.2, 5.0)
            s = self._speed
        print(f'  Speed: {s:.2f} m/s')

    def speed_down(self):
        with self._lock:
            self._speed = max(self._speed * 0.8, 0.1)
            s = self._speed
        print(f'  Speed: {s:.2f} m/s')

    def status(self):
        with self._lock:
            n = self._n
            avg = float(np.mean([np.linalg.norm(self._spd[i]) for i in range(n)]))
        print(f'  Pedestrians: {n}  avg speed: {avg:.2f} m/s')

    # ── Step ─────────────────────────────────────────────────────────────────
    def _step(self):
        with self._lock:
            n   = self._n
            pos = self._pos[:n].copy()
            spd = self._spd[:n].copy()

        # Apply maneuver then normalize all to same speed
        if self._bounds is not None:
            spd = apply_stochastic_maneuver(spd, vmag_min=self._speed*0.99,
                                             vmag_max=self._speed*1.01)
        else:
            target = self._speed * (12.0 / 1.2)
            spd = apply_stochastic_maneuver(spd, vmag_min=target*0.99,
                                             vmag_max=target*1.01)
        pos[:, 0] += spd[:, 0] * DT
        pos[:, 1] += spd[:, 1] * DT
        pos, spd = _respawn(pos, spd, self._bounds)

        with self._lock:
            self._pos[:n] = pos
            self._spd[:n] = spd

        self._publish_state(pos, spd)
        self._publish_markers(pos, spd)

    def _publish_state(self, pos, spd):
        n    = len(pos)
        data = [float(n)]
        for i in range(n):
            data += [float(pos[i,0]), float(pos[i,1]),
                     float(spd[i,0]), float(spd[i,1])]
        msg     = Float32MultiArray()
        dim     = MultiArrayDimension()
        dim.label  = 'pedestrians'
        dim.size   = len(data)
        dim.stride = 1
        msg.layout.dim.append(dim)
        msg.data = data
        self._state_pub.publish(msg)

    def _publish_markers(self, pos, spd):
        arr   = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        # Delete all previous
        d = Marker()
        d.header.frame_id = 'map'
        d.header.stamp    = stamp
        d.ns     = 'pedestrians'
        d.action = Marker.DELETEALL
        arr.markers.append(d)
        for i in range(len(pos)):
            vx, vy = spd[i]
            vmag   = math.sqrt(vx**2 + vy**2)
            theta  = math.degrees(math.atan2(vy, vx + 1e-9))
            # Ellipse size — scale with viewport
            if self._bounds is not None:
                a, b = 0.3, 0.2  # deployment: small ellipses
            else:
                a = min(2.0 + 0.2 * vmag, 4.0)
                b = min(1.0 + 0.1 * vmag, 2.0)
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = stamp
            m.ns      = 'pedestrians'
            m.id      = i
            m.type    = Marker.CYLINDER
            m.action  = Marker.ADD
            m.pose.position.x = float(pos[i, 0])
            m.pose.position.y = float(pos[i, 1])
            m.pose.position.z = 0.5
            half = math.radians(theta / 2)
            m.pose.orientation.z = math.sin(half)
            m.pose.orientation.w = math.cos(half)
            m.scale.x = 2 * a
            m.scale.y = 2 * b
            m.scale.z = 1.7
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 0.9, 0.6
            arr.markers.append(m)
        self._marker_pub.publish(arr)


# ── Keyboard thread ───────────────────────────────────────────────────────────

def _kb(node):
    try:
        import termios, tty
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        try:
            while rclpy.ok():
                ch = sys.stdin.read(1)
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                if ch in ('h', 'H'):
                    node.add_ped()
                    import time; time.sleep(0.2)
                elif ch in ('j', 'J'):
                    node.rem_ped()
                    import time; time.sleep(0.2)
                elif ch in ('u', 'U'):
                    node.speed_up()
                elif ch in ('t', 'T'):
                    node.speed_down()
                elif ch == 'p':
                    node.status()
                elif ch in ('q', 'Q', '\x03'):
                    print('  Quitting pedestrian sim...')
                    rclpy.shutdown()
                    break
                tty.setraw(fd)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except Exception as e:
        if rclpy.ok():
            node.get_logger().warn(f'KB error: {e}')


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Pedestrian simulator')
    # Read defaults from mission_waypoints.json if available
    try:
        from mission_config import get_mission
        _ped_cfg = get_mission('deployment').get('pedestrians', {}) if hasattr(get_mission('deployment'), 'get') else {}
        _ped_cfg = _ped_cfg if isinstance(_ped_cfg, dict) else {}
    except Exception:
        _ped_cfg = {}
    parser.add_argument('--n_pedestrians', type=int,   default=_ped_cfg.get('count', 8))
    parser.add_argument('--speed',         type=float, default=_ped_cfg.get('speed', 1.2))
    parser.add_argument('--hz',            type=float, default=_ped_cfg.get('hz', 25.0))
    parser.add_argument('--deploy',        action='store_true',
                        help='spawn within deployment viewport')
    args, ros_args = parser.parse_known_args()

    bounds = None
    if args.deploy:
        from mission_config import get_mission
        bounds = get_mission('deployment')['viewport']
        print(f'  Deployment mode: bounds={bounds}')

    rclpy.init(args=ros_args)
    node = PedestrianSimNode(args.n_pedestrians, args.speed, args.hz, bounds)
    print(f'  Controls: h/j add/remove pedestrians  u/t speed up/down  p status  q quit')

    kb_thread = threading.Thread(target=_kb, args=(node,), daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
