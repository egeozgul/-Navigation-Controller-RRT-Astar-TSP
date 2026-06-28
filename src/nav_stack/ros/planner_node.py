#!/usr/bin/env python3
"""
planner_node.py — ROS2 planner node: TSP global path + RRT and A* local planners.
Deployment: reads obstacle poses from MoCap (/vrpn_mocap/*/pose).
Simulation: uses hardcoded rectangular obstacles from APF_RRT_Astar.init_environment().
"""
import os
import queue
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

latched_qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray

from nav_stack.planning.APF_RRT_Astar import (
    init_environment, total_force, repulsive_force, repulsive_force_rectangles,
    attractive_force, pull_tangent_force, closest_point_and_tangent_on_polyline,
    compute_expanded_rects, extract_polyline, astar_local, euclidean_distance,
    successors,
)
from nav_stack.planning.rrt_planner_main import RRTPlanner, Point as RRTPoint
from nav_stack.planning.TSP_main import (
    bestPath, build_tsp_indices, PSO_TSP, build_full_geometric_path,
)
from nav_stack.mission.mocap_obstacles import mocap_rects_from_poses, mocap_position_to_map
from nav_stack.mission.mission_config import get_mission
from nav_stack.params.sim_reference_params import (
    V_ROBOT, DT_SIM, TOLERANCE,
    V_ROBOT_DEPLOY, DT_SIM_DEPLOY, TOLERANCE_DEPLOY,
)

PATH_TRAIL_POINTS = 100


def _tsp_compute_thread(q_start, q_goal, waypoints, rects, expanded, eps_expanded,
                        result_queue):
    """Compute TSP off the rclpy executor thread (no ROS logging here)."""
    try:
        try:
            os.nice(10)
        except OSError:
            pass

        q_start = np.asarray(q_start, dtype=float)
        q_goal = np.asarray(q_goal, dtype=float)
        waypoints = np.asarray(waypoints, dtype=float)

        traveller_sm, route_sm = bestPath(
            waypoints, q_start, q_goal, rects, expanded, eps_expanded)
        _, START, END, WP_IDX, _ = build_tsp_indices(q_start, q_goal, waypoints)
        best_path, best_cost = PSO_TSP(
            traveller_sm, START=START, END=END, WAYPOINTS=WP_IDX,
            n_particles=30, n_iter=300)
        full_path = build_full_geometric_path(best_path, route_sm)
        polyline = extract_polyline(full_path, waypoints)
        result_queue.put(('ok', full_path, polyline, float(best_cost)))
    except Exception as e:
        import traceback
        result_queue.put(('err', str(e), traceback.format_exc()))


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        # ── Simulation environment ────────────────────────────────────────────
        rect_obs, expanded, eps_expanded, _waypoints = init_environment()
        self.rect_obstacles   = rect_obs
        self.expanded_rects   = expanded
        self.eps_expanded     = eps_expanded

        # ── Missions ──────────────────────────────────────────────────────────
        self._sim_mission    = get_mission('simulation')
        self._deploy_mission = get_mission('deployment')

        # ── Robot state ───────────────────────────────────────────────────────
        self.waypoints       = self._sim_mission['waypoints']
        self._q_start        = self._sim_mission['start'].copy()
        self._q_goal_final   = self._sim_mission['goal'].copy()
        self.q_rrt           = self._q_start.copy()
        self.q_astar         = self._q_start.copy()
        self.q_goal_rrt      = self._q_goal_final.copy()
        self.q_goal_astar    = self._q_goal_final.copy()
        self.stop_rrt        = 0
        self.stop_astar      = 0
        self.done_rrt        = False
        self.done_astar      = False
        self.polyline        = None
        self.full_geometric_path = None
        self._path_idx_rrt   = 0
        self._path_idx_astar = 0
        self._min_dist_rrt   = float('inf')
        self._min_dist_astar = float('inf')

        # ── Kinematic params (defaults = simulation) ──────────────────────────
        self._deploy    = False
        self._v_robot   = V_ROBOT
        self._dt_sim    = DT_SIM
        self._tolerance = TOLERANCE

        # ── RRT planner ───────────────────────────────────────────────────────
        self.rrt = RRTPlanner(step_size=0.3, max_iterations=2000)

        # ── Pedestrian / dynamic obstacle state ───────────────────────────────
        self.obstacles_noisy  = np.empty((0, 2))
        self.obstacle_speeds  = np.empty((0, 2))
        self.got_pedestrians  = False
        self.path_rrt_hist    = []
        self.path_astar_hist  = []
        self.target_rrt       = self._q_goal_final.copy()
        self.target_astar     = self._q_goal_final.copy()

        # ── MoCap state ───────────────────────────────────────────────────────
        self._mocap_lock        = threading.Lock()
        self._mocap_seen        = False
        self._static_mocap_poses  = {}   # name → (map_x, map_y)
        self._dynamic_mocap_poses = {}
        self._mocap_applied     = False

        # ── TSP state ─────────────────────────────────────────────────────────
        self.tsp_ready               = False
        self._tsp_published          = False
        self._local_planning_logged  = False
        self._tsp_source             = 'sim'
        self._tsp_thread             = None
        self._tsp_result_queue       = None
        self._tsp_poll_timer         = None
        self._tsp_failed             = False

        # ── Publishers ────────────────────────────────────────────────────────
        self.tsp_path_pub      = self.create_publisher(Path, '/tsp_global_path', latched_qos)
        self.tsp_polyline_pub  = self.create_publisher(Path, '/tsp_global_polyline', latched_qos)
        self.pose_rrt_pub      = self.create_publisher(PoseStamped, '/robot_pose_rrt', 10)
        self.pose_astar_pub    = self.create_publisher(PoseStamped, '/robot_pose_astar', 10)
        self.path_rrt_pub      = self.create_publisher(Path, '/path_rrt', 10)
        self.path_astar_pub    = self.create_publisher(Path, '/path_astar', 10)
        self.target_rrt_pub    = self.create_publisher(PoseStamped, '/target_rrt', 10)
        self.target_astar_pub  = self.create_publisher(PoseStamped, '/target_astar', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray, '/pedestrian_state',
            self._ped_cb, 10)

        # MoCap static obstacle subscribers
        static_topics = self._deploy_mission['static_pose_topics']
        static_names  = self._deploy_mission['static_obstacle_names']
        self.get_logger().info(f'Deployment static poses: {list(static_topics)}')
        for name, topic in zip(static_names, static_topics):
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, n=name: self._static_mocap_cb(msg, n),
                qos_profile_sensor_data)

        # MoCap dynamic obstacle subscribers
        dynamic_topics = self._deploy_mission['dynamic_pose_topics']
        dynamic_names  = self._deploy_mission['dynamic_obstacle_names']
        for name, topic in zip(dynamic_names, dynamic_topics):
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, n=name: self._dynamic_mocap_cb(msg, n),
                qos_profile_sensor_data)

        # ── Timers ────────────────────────────────────────────────────────────
        self.get_logger().info(
            'Planner starting (TSP waits up to 5s for MoCap, then uses simulation rects if none)')
        self.create_timer(0.04, self._plan_step)   # 25 Hz
        self._tsp_timer = self.create_timer(5.0, self._start_tsp)

    # ── MoCap callbacks ──────────────────────────────────────────────────────
    def _static_mocap_cb(self, msg, name):
        p = msg.pose.position
        mx, my, mz = p.x, p.y, p.z
        map_x, map_y, map_z = mocap_position_to_map(mx, my, mz)
        with self._mocap_lock:
            self._static_mocap_poses[name] = (map_x, map_y)
            self._mocap_seen = True
        self.get_logger().info(
            f'Static {name}: raw ({mx:.3f}, {my:.3f}, {mz:.3f}) → map ({map_x:.3f}, {map_y:.3f})',
            once=True)

    def _dynamic_mocap_cb(self, msg, name):
        p = msg.pose.position
        map_x, map_y, _ = mocap_position_to_map(p.x, p.y, p.z)
        with self._mocap_lock:
            self._dynamic_mocap_poses[name] = (map_x, map_y)
            self._mocap_seen = True

    def _ped_cb(self, msg):
        data = np.array(msg.data, dtype=float)
        if len(data) < 4:
            return
        n = int(data[0])
        if n == 0:
            return
        pos = data[1:1 + 2*n].reshape(n, 2)
        spd = data[1 + 2*n:1 + 4*n].reshape(n, 2)
        self.obstacles_noisy = pos
        self.obstacle_speeds = spd
        self.got_pedestrians = True

    # ── Mission switching ─────────────────────────────────────────────────────
    def _apply_mission(self, mission, label):
        self._q_start      = mission['start'].copy()
        self._q_goal_final = mission['goal'].copy()
        self.waypoints     = mission['waypoints']
        self.q_rrt         = self._q_start.copy()
        self.q_astar       = self._q_start.copy()
        self.q_goal_rrt    = self._q_goal_final.copy()
        self.q_goal_astar  = self._q_goal_final.copy()
        self.stop_rrt      = 0
        self.stop_astar    = 0
        self.done_rrt      = False
        self.done_astar    = False
        self.polyline      = None
        self.full_geometric_path = None
        self._path_idx_rrt   = 0
        self._path_idx_astar = 0
        self._min_dist_rrt   = float('inf')
        self._min_dist_astar = float('inf')
        self.path_rrt_hist   = []
        self.path_astar_hist = []
        self.tsp_ready       = False
        self._tsp_published  = False
        self._local_planning_logged = False
        self._tsp_thread     = None
        self._tsp_result_queue = None
        self._tsp_failed     = False

        self._deploy = (label == 'deployment')
        if label == 'deployment':
            tol = self._deploy_mission.get('waypoint_tolerance', TOLERANCE_DEPLOY)
            self._tolerance = tol
            self._v_robot   = V_ROBOT_DEPLOY
            self._dt_sim    = DT_SIM_DEPLOY
            self.rrt = RRTPlanner(step_size=0.05, max_iterations=3000,
                                  goal_threshold=self._tolerance)
            self.rrt._deploy = True
        else:
            self._tolerance = TOLERANCE
            self._v_robot   = V_ROBOT
            self._dt_sim    = DT_SIM
            self.rrt = RRTPlanner(step_size=0.3, max_iterations=2000)

        n_wp = len(self.waypoints)
        self.get_logger().info(
            f'Mission [{label}]: start=({self._q_start[0]:.2f}, {self._q_start[1]:.2f}) '
            f'goal=({self._q_goal_final[0]:.2f}, {self._q_goal_final[1]:.2f}) '
            f'waypoints={n_wp} v={self._v_robot} tol={self._tolerance}')

    # ── Active rects ─────────────────────────────────────────────────────────
    def _active_rects(self):
        with self._mocap_lock:
            mocap_active = self._mocap_seen
            poses = dict(self._static_mocap_poses)
        if mocap_active and poses:
            poses_indexed = {}
            for name, xy in poses.items():
                try:
                    idx = int(''.join(filter(str.isdigit, name)))
                    poses_indexed[idx] = xy
                except ValueError:
                    pass
            rects, expanded, _ = mocap_rects_from_poses(poses_indexed)
            return rects, expanded
        return self.rect_obstacles, compute_expanded_rects(self.rect_obstacles, buffer=0.1)

    # ── Path segment helper ───────────────────────────────────────────────────
    def _path_segment_to_goal(self, full_path, current_pos, goal,
                               path_idx, advance_radius=0.3):
        if full_path is None or len(full_path) < 2:
            return np.array([current_pos, goal]), path_idx

        goal_dir = goal - current_pos
        goal_dist = np.linalg.norm(goal_dir)
        if goal_dist > 1e-8:
            goal_dir = goal_dir / goal_dist

        # Find end index: closest path point to goal — compute FIRST
        dists_goal = np.linalg.norm(full_path - goal, axis=1)
        end_idx = int(np.argmin(dists_goal))
        end_idx = min(end_idx, len(full_path) - 1)

        # Advance path_idx but NEVER past end_idx
        while path_idx < end_idx:
            pt = full_path[path_idx]
            too_close = np.linalg.norm(pt - current_pos) < advance_radius
            to_pt = pt - current_pos
            behind = np.dot(to_pt, goal_dir) < -0.1
            if too_close or behind:
                path_idx += 1
            else:
                break

        forward_pts = full_path[path_idx:end_idx + 1]
        if len(forward_pts) == 0:
            return np.array([current_pos, goal]), path_idx
        seg = np.vstack([current_pos, forward_pts])
        if len(seg) < 2:
            return np.array([current_pos, goal]), path_idx
        return seg, path_idx

    # ── TSP startup ──────────────────────────────────────────────────────────
    def _start_tsp(self):
        self._tsp_timer.cancel()
        with self._mocap_lock:
            mocap_on = self._mocap_seen
            poses    = dict(self._static_mocap_poses)

        if mocap_on and poses and not self._mocap_applied:
            self._mocap_applied = True
            self._apply_mission(self._deploy_mission, 'deployment')
            # Convert string keys to int indices for mocap_rects_from_poses
            poses_indexed = {}
            for name, xy in poses.items():
                try:
                    idx = int(''.join(filter(str.isdigit, name)))
                    poses_indexed[idx] = xy
                except ValueError:
                    pass
            rects, expanded, eps_expanded = mocap_rects_from_poses(poses_indexed)
            names = sorted(poses.keys())
            coords = ', '.join(f'{n}=({poses[n][0]:.2f},{poses[n][1]:.2f})' for n in names)
            self.get_logger().info(
                f'MoCap active — TSP using {len(names)} static obstacle(s): {coords}')
            self._tsp_source = 'mocap'
        else:
            self._apply_mission(self._sim_mission, 'simulation')
            rects   = self.rect_obstacles
            expanded = self.expanded_rects
            eps_expanded = self.eps_expanded
            self._tsp_source = 'sim'

        self.get_logger().info(
            f'Computing TSP global path ({self._tsp_source}, background)...')
        self._tsp_failed = False
        self._tsp_result_queue = queue.Queue(maxsize=1)
        self._tsp_thread = threading.Thread(
            target=_tsp_compute_thread,
            args=(
                self._q_start.copy(), self._q_goal_final.copy(),
                np.asarray(self.waypoints, dtype=float),
                rects, expanded, eps_expanded,
                self._tsp_result_queue,
            ),
            daemon=True,
        )
        self._tsp_thread.start()
        self._tsp_poll_timer = self.create_timer(0.25, self._poll_tsp)

    def _poll_tsp(self):
        if self._tsp_result_queue is None:
            return

        try:
            msg = self._tsp_result_queue.get_nowait()
        except queue.Empty:
            if self._tsp_thread is not None and not self._tsp_thread.is_alive():
                if self._tsp_poll_timer is not None:
                    self._tsp_poll_timer.cancel()
                    self._tsp_poll_timer = None
                if not self._tsp_failed:
                    self._tsp_failed = True
                    self.get_logger().error(
                        f'TSP thread exited unexpectedly ({self._tsp_source}).')
            return

        if self._tsp_poll_timer is not None:
            self._tsp_poll_timer.cancel()
            self._tsp_poll_timer = None
        self._tsp_thread = None
        self._tsp_result_queue = None

        if msg[0] == 'err':
            _, err, tb = msg
            self._tsp_failed = True
            self.get_logger().error(
                f'TSP failed ({self._tsp_source}): {err}\n'
                'Check deployment waypoints vs obstacle layout in mission_waypoints.json.')
            if tb:
                print(tb, file=sys.stderr)
            return

        _, full_path, polyline, best_cost = msg
        self.get_logger().info(
            f'TSP done ({self._tsp_source}). Cost={best_cost:.2f}')
        self.get_logger().info(
            f'Polyline ({len(polyline)} pts): '
            f'{[list(np.round(p, 2)) for p in polyline]}')
        self.get_logger().info(
            f'Full path ({len(full_path)} pts): '
            f'{[list(np.round(p, 2)) for p in full_path]}')

        self.full_geometric_path = full_path
        self.polyline = polyline
        self._path_idx_rrt = 0
        self._path_idx_astar = 0
        self.q_goal_rrt = polyline[2].copy()
        self.q_goal_astar = polyline[2].copy()
        self.tsp_ready = True

    # ── TSP path publisher ────────────────────────────────────────────────────
    def _try_publish_tsp_paths(self):
        if self._tsp_published or not self.tsp_ready:
            return
        self._tsp_published = True
        stamp = self.get_clock().now().to_msg()
        self._pub_path(self.tsp_path_pub, list(self.full_geometric_path), stamp)
        self._pub_path(self.tsp_polyline_pub, list(self.polyline), stamp)
        self.get_logger().info(
            f'Published /tsp_global_path ({len(self.full_geometric_path)} pts) '
            f'and /tsp_global_polyline ({len(self.polyline)} pts)')

    # ── Plan step (25 Hz) ─────────────────────────────────────────────────────
    def _plan_step(self):
        self._try_publish_tsp_paths()
        if not self.tsp_ready:
            return

        if not self._local_planning_logged:
            self._local_planning_logged = True
            with self._mocap_lock:
                mocap_on = self._mocap_seen
            static  = 'MoCap rectangles' if mocap_on else 'sim rectangles'
            n_dyn   = len(self._dynamic_mocap_poses) if mocap_on else (
                len(self.obstacles_noisy) if self.got_pedestrians else 0)
            dyn_str = f'{n_dyn} pedestrians' if not mocap_on else (
                'no dynamic obstacles' if n_dyn == 0 else f'{n_dyn} dynamic MoCap')
            self.get_logger().info(
                f'Local planning started — RRT + A* ({static}, {dyn_str})')

        with self._mocap_lock:
            mocap_on  = self._mocap_seen
            dyn_poses = dict(self._dynamic_mocap_poses)

        if mocap_on and dyn_poses:
            names = sorted(dyn_poses.keys())
            obs = np.array([dyn_poses[n] for n in names], dtype=float)
            spd = np.zeros_like(obs)
        elif self.got_pedestrians:
            obs = self.obstacles_noisy
            spd = self.obstacle_speeds
        else:
            obs = np.empty((0, 2))
            spd = np.empty((0, 2))

        rects, expanded = self._active_rects()
        self.expanded_rects = expanded
        full_path = self.full_geometric_path

        # ── RRT ───────────────────────────────────────────────────────────────
        if not self.done_rrt:
            _dist_to_goal = np.linalg.norm(self.q_rrt - self.q_goal_rrt)

            F_rrt = total_force(self.q_rrt, self.q_goal_rrt, obs, spd, rects, deploy=self._deploy)
            F1    = (repulsive_force(self.q_rrt, obs, spd, deploy=self._deploy) +
                     repulsive_force_rectangles(self.q_rrt, rects))
            _seg_rrt, self._path_idx_rrt = self._path_segment_to_goal(
                full_path, self.q_rrt, self.q_goal_rrt, self._path_idx_rrt)
            p_rrt, t_hat_rrt = closest_point_and_tangent_on_polyline(
                self.q_rrt, _seg_rrt, self.q_goal_rrt, flag=0)
            F_rrt = F_rrt + pull_tangent_force(
                self.q_rrt, p_rrt, t_hat_rrt,
                np.linalg.norm(self.q_rrt - self.q_goal_rrt))

            F_norm = np.linalg.norm(F_rrt)
            direction_rrt = F_rrt / F_norm if F_norm > 1e-8 else np.zeros(2)

            L_rrt = 0.1 if self._v_robot < 1.0 else 1.0
            q_target_local_rrt = self.q_rrt + L_rrt * direction_rrt * 0.5

            # Near waypoint: move directly with APF direction
            if _dist_to_goal < 0.5:
                self.q_rrt = self.q_rrt + self._v_robot * self._dt_sim * direction_rrt
                target_rrt = q_target_local_rrt
            else:
                rrt_path = self.rrt.plan_path(
                    start=RRTPoint(self.q_rrt[0], self.q_rrt[1]),
                    goal=RRTPoint(q_target_local_rrt[0], q_target_local_rrt[1]),
                    obstacles_noisy=obs,
                    rect_obstacles=rects,
                    obstacle_speeds=spd,
                    F=F1,
                    expanded_rects=self.expanded_rects,
                )
                if rrt_path is not None and len(rrt_path) > 1:
                    rrt_path = self.rrt.smooth_moving_average(rrt_path)
                    target_rrt = np.array([rrt_path[1].x, rrt_path[1].y])
                    vec  = target_rrt - self.q_rrt
                    dist = np.linalg.norm(vec)
                    if dist > 1e-8:
                        q_new = self.q_rrt + self._v_robot * self._dt_sim * vec / dist
                        p1 = RRTPoint(self.q_rrt[0], self.q_rrt[1])
                        p2 = RRTPoint(q_new[0], q_new[1])
                        if not self.rrt._is_path_clear(p1, p2, obs, spd):
                            q_new = target_rrt
                        self.q_rrt = q_new
                else:
                    self.q_rrt = (self.q_rrt +
                                  min(self._v_robot * self._dt_sim, self.rrt.step_size) *
                                  direction_rrt)
                    target_rrt = q_target_local_rrt

            self.target_rrt = target_rrt.copy()
            self.path_rrt_hist.append(self.q_rrt.copy())
            if len(self.path_rrt_hist) > PATH_TRAIL_POINTS:
                self.path_rrt_hist = self.path_rrt_hist[-PATH_TRAIL_POINTS:]

            _dist_to_goal = np.linalg.norm(self.q_rrt - self.q_goal_rrt)
            if not hasattr(self, '_last_log_dist_rrt') or \
               abs(getattr(self, '_last_log_dist_rrt', 9999) - _dist_to_goal) > 0.1:
                self._last_log_dist_rrt = _dist_to_goal
                self.get_logger().info(
                    f'RRT dist={_dist_to_goal:.3f} '
                    f'wp={self.stop_rrt+1}/{len(self.polyline)-3} '
                    f'goal={np.round(self.q_goal_rrt,2)} '
                    f'pos={np.round(self.q_rrt,2)} '
                    f'path_idx={self._path_idx_rrt}')

            if _dist_to_goal < self._tolerance:
                self.stop_rrt += 1
                if self.stop_rrt + 2 < len(self.polyline):
                    self.q_goal_rrt = self.polyline[2 + self.stop_rrt].copy()
                    if self.full_geometric_path is not None:
                        prev_wp = self.polyline[self.stop_rrt + 1]
                        dists = np.linalg.norm(
                            self.full_geometric_path - prev_wp, axis=1)
                        self._path_idx_rrt = int(np.argmin(dists))
                    self._min_dist_rrt = float('inf')
                    self.get_logger().info(
                        f'RRT → waypoint {self.stop_rrt}: {self.q_goal_rrt} '
                        f'path_idx={self._path_idx_rrt}')
                else:
                    self.q_goal_rrt = self._q_goal_final.copy()
                    self.get_logger().info(
                        f'RRT → final goal: {self.q_goal_rrt}')

            if (not self.done_rrt
                    and np.linalg.norm(self.q_rrt - self._q_goal_final) < self._tolerance
                    and np.linalg.norm(self.q_goal_rrt - self._q_goal_final) < self._tolerance):
                self.done_rrt = True
                self.get_logger().info('RRT reached final goal')

        # ── A* ────────────────────────────────────────────────────────────────
        if not self.done_astar:
            F_astar = total_force(self.q_astar, self.q_goal_astar, obs, spd, rects, deploy=self._deploy)
            _seg_astar, self._path_idx_astar = self._path_segment_to_goal(
                full_path, self.q_astar, self.q_goal_astar, self._path_idx_astar)
            p_astar, t_hat_astar = closest_point_and_tangent_on_polyline(
                self.q_astar, _seg_astar, self.q_goal_astar, flag=0)
            F_astar = F_astar + pull_tangent_force(
                self.q_astar, p_astar, t_hat_astar,
                np.linalg.norm(self.q_astar - self.q_goal_astar))

            F_norm = np.linalg.norm(F_astar)
            direction_astar = F_astar / F_norm if F_norm > 1e-8 else np.zeros(2)

            L_astar = 0.3 if self._v_robot < 1.0 else 3.0
            q_target_local_astar = self.q_astar + L_astar * direction_astar

            # Near waypoint: move directly with APF direction
            if np.linalg.norm(self.q_astar - self.q_goal_astar) < 0.5:
                self.q_astar = self.q_astar + self._v_robot * self._dt_sim * direction_astar
                target_astar = q_target_local_astar
            else:
                xmin, xmax = self.q_astar[0] - 5, self.q_astar[0] + 5
                ymin, ymax = self.q_astar[1] - 5, self.q_astar[1] + 5
                q_target_local_astar = np.clip(
                    q_target_local_astar, [xmin, ymin], [xmax, ymax])
                astar_path = astar_local(
                    start_state=tuple(self.q_astar),
                    goal_state=tuple(q_target_local_astar),
                    heuristic_func=euclidean_distance,
                    successors_func=successors,
                    obstacles_noisy=obs,
                    obstacle_speeds=spd,
                    rect_obstacles=rects,
                    bounds=(xmin, xmax, ymin, ymax),
                    step=0.5,
                )
                if astar_path is not None and len(astar_path) > 1:
                    target_astar = np.array([astar_path[1][0], astar_path[1][1]])
                    vec  = target_astar - self.q_astar
                    dist = np.linalg.norm(vec)
                    if dist > 1e-8:
                        self.q_astar = (self.q_astar +
                                        self._v_robot * self._dt_sim * vec / dist)
                else:
                    self.q_astar = (self.q_astar +
                                    self._v_robot * self._dt_sim * direction_astar)
                    target_astar = q_target_local_astar

            self.target_astar = target_astar.copy()
            self.path_astar_hist.append(self.q_astar.copy())
            if len(self.path_astar_hist) > PATH_TRAIL_POINTS:
                self.path_astar_hist = self.path_astar_hist[-PATH_TRAIL_POINTS:]

            _dist_astar = np.linalg.norm(self.q_astar - self.q_goal_astar)
            if _dist_astar < self._tolerance:
                self.stop_astar += 1
                if self.stop_astar + 2 < len(self.polyline):
                    self.q_goal_astar = self.polyline[2 + self.stop_astar].copy()
                    if self.full_geometric_path is not None:
                        prev_wp = self.polyline[self.stop_astar + 1]
                        dists = np.linalg.norm(
                            self.full_geometric_path - prev_wp, axis=1)
                        self._path_idx_astar = int(np.argmin(dists))
                    self._min_dist_astar = float('inf')
                    self.get_logger().info(
                        f'A* → waypoint {self.stop_astar}: {self.q_goal_astar} '
                        f'path_idx={self._path_idx_astar}')
                else:
                    self.q_goal_astar = self._q_goal_final.copy()
                    self.get_logger().info(
                        f'A* → final goal: {self.q_goal_astar}')

            if (not self.done_astar
                    and np.linalg.norm(self.q_astar - self._q_goal_final) < self._tolerance
                    and np.linalg.norm(self.q_goal_astar - self._q_goal_final) < self._tolerance):
                self.done_astar = True
                self.get_logger().info('A* reached final goal')

        # ── Publish ───────────────────────────────────────────────────────────
        stamp = self.get_clock().now().to_msg()
        self._pub_pose(self.pose_rrt_pub, self.q_rrt, stamp)
        self._pub_pose(self.pose_astar_pub, self.q_astar, stamp)
        self._pub_pose(self.target_rrt_pub, self.target_rrt, stamp)
        self._pub_pose(self.target_astar_pub, self.target_astar, stamp)
        if len(self.path_rrt_hist) > 1:
            self._pub_path(self.path_rrt_pub, self.path_rrt_hist, stamp)
        if len(self.path_astar_hist) > 1:
            self._pub_path(self.path_astar_pub, self.path_astar_hist, stamp)


    # ── Helpers ───────────────────────────────────────────────────────────────
    def _pub_pose(self, pub, pos, stamp):
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.orientation.w = 1.0
        pub.publish(msg)

    def _pub_path(self, pub, pts, stamp):
        msg = Path()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        for pt in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(pt[0]) if hasattr(pt, '__len__') else float(pt.x)
            ps.pose.position.y = float(pt[1]) if hasattr(pt, '__len__') else float(pt.y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    print('  Planner node running — press q to quit')

    def _kb(node):
        if not sys.stdin.isatty():
            return
        try:
            print("  Controls: type 'q' + Enter to quit")
            for line in sys.stdin:
                if line.strip().lower() in ('q', 'quit'):
                    print('  Quitting planner...')
                    rclpy.shutdown()
                    break
        except Exception as e:
            node.get_logger().warn(f'KB error: {e}')

    kb_thread = threading.Thread(target=_kb, args=(node,), daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
