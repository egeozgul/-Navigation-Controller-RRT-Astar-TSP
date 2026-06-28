#!/usr/bin/env python3
"""
visualizer_node.py
Subscribes to all planner and pedestrian topics and renders them
in a live matplotlib animation — equivalent to the current simulation
plot but driven by ROS2 topics instead of internal state.

Topics subscribed:
  /pedestrian_state   — pedestrian positions + velocities
  /static_obstacles   — simulation rectangles (when MoCap not active)
  /vrpn_mocap/obstacle{1,2,3}/pose — live MoCap obstacles (when publishing)
  /robot_pose_rrt     — RRT robot position
  /robot_pose_astar   — A* robot position
  /path_rrt           — full RRT path history
  /path_astar         — full A* path history
  /tsp_global_path      — full TSP geometric path (latched)
  /tsp_global_polyline  — TSP polyline through destination points (latched)
  /cmd_goal_rrt       — TSP waypoint target (RRT)
  /cmd_goal_astar     — TSP waypoint target (A*)

Field overlay (left panel — potential and vector are independent):
  Potential: Potential RRT | Potential A* | Off
  Vector:    Vector RRT    | Vector A*    | Off
"""
import math
import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Rectangle
from matplotlib.widgets import RadioButtons

from nav_stack.planning.APF_RRT_Astar import init_environment, potential, total_force, compute_expanded_rects
from nav_stack.params.sim_reference_params import (
    ped_ellipse_axes, ped_collision_ellipse_axes, ped_apf_model, ped_heading_deg,
)
from nav_stack.mission.mission_config import get_mission
from nav_stack.mission.mocap_obstacles import (
    MOCAP_POSE_TOPICS, MOCAP_DYNAMIC_POSE_TOPICS, MOCAP_STATIC_NAMES,
    MOCAP_DYNAMIC_NAMES, MOCAP_OBSTACLE_COUNT,
    MOCAP_OBSTACLE_SIZES, MOCAP_OBSTACLE_COLORS,
    MOCAP_LABEL_ANGLES_DEG, MOCAP_LABEL_OFFSET,
    pose_xy, poses_to_rects, center_to_rect, sim_expanded_rects,
)

COLLISION_RECT_BUFFER = 0.1

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray

POT_LABELS = ("Potential RRT", "Potential A*", "Off")
VEC_LABELS = ("Vector RRT", "Vector A*", "Off")
PATH_TRAIL_POINTS = 100
POT_LABEL_TO_MODE = {lbl: mode for lbl, mode in zip(POT_LABELS, ("rrt", "astar", "off"))}
VEC_LABEL_TO_MODE = {lbl: mode for lbl, mode in zip(VEC_LABELS, ("rrt", "astar", "off"))}

_PANEL_BORDER = "#d0d7de"
_PANEL_TITLE = "#111827"
_COLOR_RRT = "#dc2626"
_COLOR_ASTAR = "#2563eb"
_COLOR_OFF = "#6b7280"
_RADIO_POT = "#2563eb"
_RADIO_VEC = "#dc2626"


def _tail_path(pts, n=PATH_TRAIL_POINTS):
    """Keep only the most recent trail segment for display."""
    if len(pts) <= n:
        return pts
    return pts[-n:]


def _label_color(text):
    if text.endswith("RRT"):
        return _COLOR_RRT
    if text.endswith("A*"):
        return _COLOR_ASTAR
    return _COLOR_OFF


def _style_radio(radio, activecolor):
    radio.activecolor = activecolor
    for circle in radio.circles:
        circle.set_edgecolor(_PANEL_BORDER)
        circle.set_linewidth(1.8)
    for lbl in radio.labels:
        lbl.set_fontsize(15)
        lbl.set_color(_label_color(lbl.get_text()))
        lbl.set_fontfamily("sans-serif")


def _highlight_radio(radio, active_label):
    for lbl in radio.labels:
        on = lbl.get_text() == active_label
        lbl.set_fontweight("600" if on else "normal")
        lbl.set_color(_label_color(lbl.get_text()))


def _layout_radio(radio, top=0.66, bottom=0.04):
    n = len(radio.labels)
    ys = np.linspace(top, bottom, n)
    xs_circle, xs_label = 0.10, 0.30
    radius = min(0.044, 0.80 / max(n, 1) * 0.14)
    for y, circle, lbl in zip(ys, radio.circles, radio.labels):
        circle.center = (xs_circle, y)
        circle.set_radius(radius)
        lbl.set_position((xs_label, y))
        lbl.set_verticalalignment("center")


def _prep_field_section(ax, title):
    ax.set_facecolor("#fafbfc")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.margins(0)
    for spine in ax.spines.values():
        spine.set_edgecolor(_PANEL_BORDER)
        spine.set_linewidth(0.8)
    ax.text(
        0.5, 0.97, title,
        ha="center", va="top", fontsize=15, fontweight="600",
        color=_PANEL_TITLE, transform=ax.transAxes,
    )


def _build_field_panel(fig, field_state, on_select):
    """Compact sidebar: separate potential + vector selectors."""
    gs = fig.add_gridspec(
        1, 2, width_ratios=(0.20, 1), wspace=0.03,
        left=0.02, right=0.99, top=0.97, bottom=0.05,
    )
    # Two panels stacked at the top; remaining sidebar space stays empty.
    gs_side = gs[0, 0].subgridspec(3, 1, height_ratios=(2, 2, 4), hspace=0.04)
    pot_ax = fig.add_subplot(gs_side[0, 0])
    vec_ax = fig.add_subplot(gs_side[1, 0])
    plot_ax = fig.add_subplot(gs[0, 1])

    _prep_field_section(pot_ax, "Potential")
    _prep_field_section(vec_ax, "Vector")

    pot_radio = RadioButtons(pot_ax, POT_LABELS, active=2)
    vec_radio = RadioButtons(vec_ax, VEC_LABELS, active=2)
    _style_radio(pot_radio, _RADIO_POT)
    _style_radio(vec_radio, _RADIO_VEC)

    def _layout_all(_event=None):
        _layout_radio(pot_radio)
        _layout_radio(vec_radio)
        fig.canvas.draw_idle()

    def _on_pot(label):
        field_state["pot"] = POT_LABEL_TO_MODE[label]
        _highlight_radio(pot_radio, label)
        on_select()

    def _on_vec(label):
        field_state["vec"] = VEC_LABEL_TO_MODE[label]
        _highlight_radio(vec_radio, label)
        on_select()

    pot_radio.on_clicked(_on_pot)
    vec_radio.on_clicked(_on_vec)
    _highlight_radio(pot_radio, POT_LABELS[2])
    _highlight_radio(vec_radio, VEC_LABELS[2])
    _layout_all()
    fig.canvas.mpl_connect("resize_event", _layout_all)

    return plot_ax


# ── Shared state (written by ROS callbacks, read by matplotlib) ───────────────
STATE = None  # set after mission helpers below


def _sim_mission():
    return get_mission('simulation')


def _deploy_mission():
    return get_mission('deployment')


def _ped_deploy_mode(using_mocap: bool, positions: np.ndarray, n_peds: int) -> bool:
    """True when pedestrians should use the uniform deployment ellipse model."""
    if using_mocap:
        return True
    if n_peds <= 0:
        return False
    vp = _deploy_mission().get('viewport')
    if not vp:
        return False
    pts = np.asarray(positions, dtype=float)[:n_peds]
    return bool(np.all(
        (pts[:, 0] >= vp['x_min']) & (pts[:, 0] <= vp['x_max']) &
        (pts[:, 1] >= vp['y_min']) & (pts[:, 1] <= vp['y_max'])
    ))


def _looks_like_sim_start(pt, tol=1.0):
    sim_start = _sim_mission()['start']
    deploy_start = _deploy_mission()['start']
    if np.allclose(sim_start, deploy_start, atol=tol):
        return False
    return np.allclose(np.asarray(pt, dtype=float), sim_start, atol=tol)


def _apply_deployment_state():
    """Clear simulation defaults when switching to MoCap / deployment view."""
    deploy = _deploy_mission()
    start = deploy['start'].copy()
    goal = deploy['goal'].copy()
    STATE.q_rrt = start.copy()
    STATE.q_astar = start.copy()
    STATE.path_rrt = [start.copy()]
    STATE.path_astar = [start.copy()]
    STATE.goal_rrt = goal.copy()
    STATE.goal_astar = goal.copy()
    STATE.tsp_global = []
    STATE.tsp_polyline = []
    STATE.rect_obstacles = []
    STATE.mocap_poses = {}
    STATE.dynamic_mocap_poses = {}
    STATE.deployment_mode_applied = True


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        sim = _sim_mission()
        start = sim['start'].copy()
        goal = sim['goal'].copy()

        # pedestrians
        self.ped_positions = np.zeros((0, 2))
        self.ped_speeds    = np.zeros((12, 2))
        self.n_peds        = 0

        # static obstacles (sim rectangles from /static_obstacles)
        self.rect_obstacles = []
        # MoCap live obstacles (from /vrpn_mocap/obstacle*/pose)
        self.mocap_poses = {}   # static index 1..N → (x, y)
        self.dynamic_mocap_poses = {}  # name → (x, y)
        self.mocap_seen = False
        self.deployment_mode_applied = False

        # robot poses
        self.q_rrt         = start.copy()
        self.q_astar       = start.copy()

        # path histories (full trail from planner)
        self.path_rrt      = [start.copy()]
        self.path_astar    = [start.copy()]
        self.tsp_global    = []
        self.tsp_polyline  = []

        # goals (TSP waypoints)
        self.goal_rrt      = goal.copy()
        self.goal_astar    = goal.copy()

        # TSP waypoints (sim vs deployment chosen at draw time)
        self.waypoints     = sim['waypoints']


STATE = SharedState()


class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        qos = QoSProfile(depth=10)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(Float32MultiArray, '/pedestrian_state',
                                 self._ped_cb,        qos)
        self.create_subscription(MarkerArray,        '/static_obstacles',
                                 self._static_cb,     latched)
        self.create_subscription(PoseStamped,        '/robot_pose_rrt',
                                 self._pose_rrt_cb,   qos)
        self.create_subscription(PoseStamped,        '/robot_pose_astar',
                                 self._pose_astar_cb, qos)
        self.create_subscription(Path,               '/path_rrt',
                                 self._path_rrt_cb,   qos)
        self.create_subscription(Path,               '/path_astar',
                                 self._path_astar_cb, qos)
        self.create_subscription(PoseStamped,        '/cmd_goal_rrt',
                                 self._goal_rrt_cb,   qos)
        self.create_subscription(PoseStamped,        '/cmd_goal_astar',
                                 self._goal_astar_cb, qos)
        self.create_subscription(Path,               '/tsp_global_path',
                                 self._tsp_global_cb, latched)
        self.create_subscription(Path,               '/tsp_global_polyline',
                                 self._tsp_polyline_cb, latched)

        self._mocap_logged = set()
        self._dyn_mocap_logged = set()

        for idx, topic in enumerate(MOCAP_POSE_TOPICS, start=1):
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, i=idx: self._static_mocap_cb(msg, i),
                qos_profile_sensor_data,
            )
        for name, topic in zip(MOCAP_DYNAMIC_NAMES, MOCAP_DYNAMIC_POSE_TOPICS):
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, n=name: self._dynamic_mocap_cb(msg, n),
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            'Visualizer ready — deployment static/dynamic poses from mission_waypoints.json')

    def _ped_cb(self, msg):
        data = list(msg.data)
        n    = int(data[0])
        pos  = np.zeros((n, 2))
        spd  = np.zeros((n, 2))
        for i in range(n):
            base = 1 + i * 4
            pos[i] = [data[base],   data[base+1]]
            spd[i] = [data[base+2], data[base+3]]
        with STATE.lock:
            STATE.ped_positions = pos
            STATE.ped_speeds    = spd
            STATE.n_peds        = n

    def _static_cb(self, msg):
        rects = []
        for m in msg.markers:
            if m.ns == 'static_obstacles':
                x = m.pose.position.x - m.scale.x / 2
                y = m.pose.position.y - m.scale.y / 2
                rects.append([x, y, m.scale.x, m.scale.y])
        with STATE.lock:
            if STATE.mocap_seen:
                return
            STATE.rect_obstacles = rects

    def _static_mocap_cb(self, msg, index: int):
        p = msg.pose.position
        x, y = pose_xy(msg)
        with STATE.lock:
            if not STATE.deployment_mode_applied:
                _apply_deployment_state()
            STATE.mocap_poses[index] = (x, y)
            STATE.mocap_seen = True
        if index not in self._mocap_logged:
            self._mocap_logged.add(index)
            label = (MOCAP_STATIC_NAMES[index - 1]
                     if index - 1 < len(MOCAP_STATIC_NAMES) else str(index))
            self.get_logger().info(
                f'Static {label}: raw ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) '
                f'→ map ({x:.2f}, {y:.2f})')
            if index == 1:
                self.get_logger().info(
                    'Deployment mode — cleared sim robot/TSP markers')

    def _dynamic_mocap_cb(self, msg, name: str):
        p = msg.pose.position
        x, y = pose_xy(msg)
        with STATE.lock:
            if not STATE.deployment_mode_applied:
                _apply_deployment_state()
            STATE.dynamic_mocap_poses[name] = (x, y)
            STATE.mocap_seen = True
        if name not in self._dyn_mocap_logged:
            self._dyn_mocap_logged.add(name)
            self.get_logger().info(
                f'Dynamic {name}: raw ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) '
                f'→ map ({x:.2f}, {y:.2f})')

    def _pose_rrt_cb(self, msg):
        pt = [msg.pose.position.x, msg.pose.position.y]
        with STATE.lock:
            if STATE.mocap_seen and _looks_like_sim_start(pt):
                return
            STATE.q_rrt = np.array(pt)

    def _pose_astar_cb(self, msg):
        pt = [msg.pose.position.x, msg.pose.position.y]
        with STATE.lock:
            if STATE.mocap_seen and _looks_like_sim_start(pt):
                return
            STATE.q_astar = np.array(pt)

    def _path_rrt_cb(self, msg):
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        if not pts:
            return
        with STATE.lock:
            if STATE.mocap_seen and _looks_like_sim_start(pts[0]):
                return
            STATE.path_rrt = _tail_path(pts)

    def _path_astar_cb(self, msg):
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        if not pts:
            return
        with STATE.lock:
            if STATE.mocap_seen and _looks_like_sim_start(pts[0]):
                return
            STATE.path_astar = _tail_path(pts)

    def _goal_rrt_cb(self, msg):
        pt = [msg.pose.position.x, msg.pose.position.y]
        with STATE.lock:
            if STATE.mocap_seen and _looks_like_sim_start(pt):
                return
            STATE.goal_rrt = np.array(pt)

    def _goal_astar_cb(self, msg):
        pt = [msg.pose.position.x, msg.pose.position.y]
        with STATE.lock:
            if STATE.mocap_seen and _looks_like_sim_start(pt):
                return
            STATE.goal_astar = np.array(pt)

    def _tsp_global_cb(self, msg):
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        if not pts:
            return
        with STATE.lock:
            if STATE.mocap_seen and _looks_like_sim_start(pts[0]):
                return
            STATE.tsp_global = pts
        self.get_logger().info(f'Global TSP path received ({len(pts)} points)')

    def _tsp_polyline_cb(self, msg):
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        if not pts:
            return
        with STATE.lock:
            if STATE.mocap_seen and _looks_like_sim_start(pts[0]):
                return
            STATE.tsp_polyline = pts
        self.get_logger().info(f'TSP polyline received ({len(pts)} waypoints)')


# ── Matplotlib animation ──────────────────────────────────────────────────────

def run_visualizer():
    fig = plt.figure(figsize=(9.6, 9), dpi=80, facecolor="white")
    field_state = {"pot": "off", "vec": "off"}

    def _request_redraw():
        fig.canvas.draw_idle()

    ax = _build_field_panel(fig, field_state, _request_redraw)

    _sim_mission = get_mission('simulation')
    _deploy_mission = get_mission('deployment')

    def _apply_viewport(mission):
        x_min, x_max, y_min, y_max = mission['viewport']
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)

    _apply_viewport(_sim_mission)
    ax.set_aspect('equal')
    ax.set_title("ROS2 Planner Visualizer")
    ax.set_facecolor('white')
    ax.grid(color='gray', linestyle='--', linewidth=0.4, alpha=0.5)

    def _scatter_mission(ax, mission, label, color):
        wps = mission['waypoints']
        if len(wps):
            offsets = wps
        else:
            offsets = np.empty((0, 2))
        return ax.scatter(
            offsets[:, 0], offsets[:, 1],
            color=color, marker='*', s=150, zorder=5, label=label)

    # Two independent marker sets — only one visible at a time
    sim_waypoint_scatter = _scatter_mission(
        ax, _sim_mission, 'Sim waypoints', 'purple')
    deploy_waypoint_scatter = _scatter_mission(
        ax, _deploy_mission, 'Deploy waypoints', '#d97706')

    sim_start_dot, = ax.plot(
        _sim_mission['start'][0], _sim_mission['start'][1],
        'go', markersize=10, label='Sim start', zorder=5)
    sim_goal_dot, = ax.plot(
        _sim_mission['goal'][0], _sim_mission['goal'][1],
        'r*', markersize=12, label='Sim goal', zorder=5)

    deploy_start_dot, = ax.plot(
        _deploy_mission['start'][0], _deploy_mission['start'][1],
        'o', color='#059669', markersize=10, label='Deploy start', zorder=5)
    deploy_goal_dot, = ax.plot(
        _deploy_mission['goal'][0], _deploy_mission['goal'][1],
        '*', color='#ea580c', markersize=12, label='Deploy goal', zorder=5)

    deploy_waypoint_scatter.set_visible(False)
    deploy_start_dot.set_visible(False)
    deploy_goal_dot.set_visible(False)

    # global TSP path + robot trails
    tsp_global_line, = ax.plot([], [], color='#7c3aed', lw=2.5,
                               alpha=0.75, linestyle='-', zorder=3,
                               label='TSP global path')
    tsp_polyline_line, = ax.plot([], [], color='#c4b5fd', lw=2.0,
                                 alpha=0.9, linestyle='--', zorder=3,
                                 label='TSP destinations')
    path_rrt_line,   = ax.plot([], [], color='#ef4444', lw=2.0,
                                alpha=0.85, label='RRT path')
    path_astar_line, = ax.plot([], [], color='#3b82f6', lw=2.0,
                                alpha=0.85, label='A* path')

    # robot dots
    robot_rrt_dot,   = ax.plot([], [], 'ro', markersize=8, zorder=6)
    robot_astar_dot, = ax.plot([], [], 'bo', markersize=8, zorder=6)

    goal_rrt_dot,   = ax.plot([], [], 'r^', markersize=9, zorder=6, label='RRT goal')
    goal_astar_dot, = ax.plot([], [], 'b^', markersize=9, zorder=6, label='A* goal')

    # pedestrian scatter
    ped_scatter = ax.scatter([], [], c='cyan', edgecolors='black',
                              s=60, zorder=4, label='Pedestrians')

    ellipse_patches = []
    collision_ellipse_patches = []
    apf_influence_ellipse_patches = []
    rect_patches    = []
    expanded_rect_patches = []
    mocap_label_artists = []
    mocap_center_artists = []
    mocap_status_text = None
    pf_artists      = []
    PF_HEAT_GRID    = 20
    PF_VEC_GRID     = 16
    _default_rects, _, _, _ = init_environment()

    def _ensure_ellipse(i, edgecolor='black', facecolor='cyan'):
        while len(ellipse_patches) <= i:
            ell = Ellipse(
                xy=(0, 0), width=0.1, height=0.1, angle=0,
                edgecolor='black', facecolor='cyan',
                alpha=0.35, linestyle='--', linewidth=1.2,
            )
            ell.set_visible(False)
            ax.add_patch(ell)
            ellipse_patches.append(ell)
        ell = ellipse_patches[i]
        ell.set_edgecolor(edgecolor)
        ell.set_facecolor(facecolor)
        return ell

    def _hide_extra_ellipses(visible_count):
        for j in range(visible_count, len(ellipse_patches)):
            ellipse_patches[j].set_visible(False)

    def _ensure_collision_ellipse(i):
        while len(collision_ellipse_patches) <= i:
            ell = Ellipse(
                xy=(0, 0), width=0.1, height=0.1, angle=0,
                edgecolor='#ea580c', facecolor='none',
                alpha=0.95, linestyle='--', linewidth=1.5,
                zorder=5,
            )
            ell.set_visible(False)
            ax.add_patch(ell)
            collision_ellipse_patches.append(ell)
        return collision_ellipse_patches[i]

    def _hide_extra_collision_ellipses(visible_count):
        for j in range(visible_count, len(collision_ellipse_patches)):
            collision_ellipse_patches[j].set_visible(False)

    def _ensure_apf_influence_ellipse(i):
        while len(apf_influence_ellipse_patches) <= i:
            ell = Ellipse(
                xy=(0, 0), width=0.1, height=0.1, angle=0,
                edgecolor='#7c3aed', facecolor='#7c3aed',
                alpha=0.22, linestyle=':', linewidth=1.2,
                zorder=4,
            )
            ell.set_visible(False)
            ax.add_patch(ell)
            apf_influence_ellipse_patches.append(ell)
        return apf_influence_ellipse_patches[i]

    def _hide_extra_apf_influence_ellipses(visible_count):
        for j in range(visible_count, len(apf_influence_ellipse_patches)):
            apf_influence_ellipse_patches[j].set_visible(False)

    def _clear_pf():
        for artist in pf_artists:
            try:
                artist.remove()
            except Exception:
                pass
        pf_artists.clear()

    def _draw_heatmap(goal, obs, spd, rects, deploy=False):
        grid = PF_HEAT_GRID
        xs = np.linspace(ax.get_xlim()[0], ax.get_xlim()[1], grid)
        ys = np.linspace(ax.get_ylim()[0], ax.get_ylim()[1], grid)
        xx, yy = np.meshgrid(xs, ys)
        zz = np.zeros_like(xx)
        for ri in range(grid):
            for ci in range(grid):
                q = np.array([xx[ri, ci], yy[ri, ci]])
                zz[ri, ci] = potential(q, goal, obs, spd, deploy=deploy)
        zz = np.clip(zz, np.percentile(zz, 5), np.percentile(zz, 95))
        cf = ax.contourf(xx, yy, zz, levels=24, cmap="RdYlGn_r", alpha=0.35, zorder=0)
        pf_artists.extend(cf.collections)

    def _draw_vectors(goal, obs, spd, rects, color, deploy=False):
        qgrid = PF_VEC_GRID
        qxs = np.linspace(ax.get_xlim()[0], ax.get_xlim()[1], qgrid)
        qys = np.linspace(ax.get_ylim()[0], ax.get_ylim()[1], qgrid)
        qx, qy = np.meshgrid(qxs, qys)
        fx = np.zeros_like(qx)
        fy = np.zeros_like(qy)
        for ri in range(qgrid):
            for ci in range(qgrid):
                q = np.array([qx[ri, ci], qy[ri, ci]])
                f = total_force(q, goal, obs, spd, rects, deploy=deploy)
                mag = np.linalg.norm(f) + 1e-9
                fx[ri, ci] = f[0] / mag
                fy[ri, ci] = f[1] / mag
        qv = ax.quiver(qx, qy, fx, fy, color=color, alpha=0.6,
                       scale=qgrid*0.8/max(ax.get_xlim()[1]-ax.get_xlim()[0],1), width=0.003*50/max(ax.get_xlim()[1]-ax.get_xlim()[0],1), zorder=2)
        pf_artists.append(qv)

    def _draw_pf(obs, spd, goal_rrt, goal_astar, rects, pot_mode, vec_mode, deploy=False):
        _clear_pf()
        if pot_mode == "off" and vec_mode == "off":
            return
        if not rects:
            rects = _default_rects
        obs = np.asarray(obs)
        spd = np.asarray(spd)

        if pot_mode == "rrt":
            _draw_heatmap(goal_rrt, obs, spd, rects, deploy=deploy)
        elif pot_mode == "astar":
            _draw_heatmap(goal_astar, obs, spd, rects, deploy=deploy)

        if vec_mode == "rrt":
            _draw_vectors(goal_rrt, obs, spd, rects, color="crimson", deploy=deploy)
        elif vec_mode == "astar":
            _draw_vectors(goal_astar, obs, spd, rects, color="royalblue", deploy=deploy)

    ax.legend(loc='upper right', fontsize=7, framealpha=0.92)

    def update(frame):
        with STATE.lock:
            pos   = STATE.ped_positions.copy()
            spd   = STATE.ped_speeds.copy()
            n     = STATE.n_peds
            rects = list(STATE.rect_obstacles)
            mocap_poses = dict(STATE.mocap_poses)
            dynamic_mocap_poses = dict(STATE.dynamic_mocap_poses)
            mocap_seen = STATE.mocap_seen
            q_rrt   = STATE.q_rrt.copy()
            q_astar = STATE.q_astar.copy()
            p_rrt   = list(STATE.path_rrt)
            p_astar = list(STATE.path_astar)
            g_rrt   = STATE.goal_rrt.copy()
            g_astar = STATE.goal_astar.copy()
            tsp_g   = list(STATE.tsp_global)
            tsp_pl  = list(STATE.tsp_polyline)

        # Prefer live MoCap rectangles when VRPN is publishing
        using_mocap = bool(mocap_seen)
        if using_mocap and mocap_poses:
            display_rects = poses_to_rects(mocap_poses)
        elif using_mocap:
            display_rects = []
        else:
            display_rects = rects if rects else _default_rects

        sim_waypoint_scatter.set_visible(not using_mocap)
        sim_start_dot.set_visible(not using_mocap)
        sim_goal_dot.set_visible(not using_mocap)
        deploy_waypoint_scatter.set_visible(using_mocap)
        deploy_start_dot.set_visible(using_mocap)
        deploy_goal_dot.set_visible(using_mocap)
        _apply_viewport(_deploy_mission if using_mocap else _sim_mission)

        ped_deploy = _ped_deploy_mode(using_mocap, pos, n)

        # ── field overlay (includes pedestrian APF; skip when both off) ──
        pf_obs = pos[:n].copy() if n > 0 else np.empty((0, 2))
        pf_spd = spd[:n].copy() if n > 0 else np.empty((0, 2))
        pot_on = field_state["pot"] != "off"
        vec_on = field_state["vec"] != "off"
        if pot_on or vec_on:
            if not hasattr(update, '_pf_frame'):
                update._pf_frame = 0
            update._pf_frame += 1
            if update._pf_frame % 5 == 0:
                _draw_pf(pf_obs, pf_spd, g_rrt, g_astar, display_rects,
                         field_state["pot"], field_state["vec"], deploy=ped_deploy)

        if len(tsp_g) > 1 and not (using_mocap and _looks_like_sim_start(tsp_g[0])):
            arr_tsp = np.array(tsp_g)
            tsp_global_line.set_data(arr_tsp[:, 0], arr_tsp[:, 1])
        else:
            tsp_global_line.set_data([], [])

        if len(tsp_pl) > 1 and not (using_mocap and _looks_like_sim_start(tsp_pl[0])):
            arr_pl = np.array(tsp_pl)
            tsp_polyline_line.set_data(arr_pl[:, 0], arr_pl[:, 1])
        else:
            tsp_polyline_line.set_data([], [])

        # ── pedestrian ellipses (reuse patches — no per-frame recreate) ──
        if using_mocap and dynamic_mocap_poses:
            dyn_names = sorted(dynamic_mocap_poses.keys())
            dyn_pts = np.array([dynamic_mocap_poses[k] for k in dyn_names], dtype=float)
            ped_scatter.set_offsets(dyn_pts)
            for i, _name in enumerate(dyn_names):
                ell = _ensure_ellipse(i, edgecolor='#0891b2', facecolor='cyan')
                ell.set_center((dyn_pts[i, 0], dyn_pts[i, 1]))
                ell.width = 1.2
                ell.height = 1.2
                ell.angle = 0.0
                ell.set_visible(True)

                ca, cb = ped_collision_ellipse_axes(0.0, i, deploy=True)
                coll = _ensure_collision_ellipse(i)
                coll.set_center((dyn_pts[i, 0], dyn_pts[i, 1]))
                coll.width = 2 * ca
                coll.height = 2 * cb
                coll.angle = 0.0
                coll.set_visible(True)

                _sz, (_, _), (ia, ib), strength = ped_apf_model(0.0, i, deploy=True)
                infl = _ensure_apf_influence_ellipse(i)
                infl.set_center((dyn_pts[i, 0], dyn_pts[i, 1]))
                infl.width = 2 * ia
                infl.height = 2 * ib
                infl.angle = 0.0
                infl.set_alpha(0.12 + 0.28 * strength)
                infl.set_linewidth(1.0 + 1.2 * strength)
                if i == 0:
                    infl.set_label('APF influence')
                infl.set_visible(True)
            _hide_extra_ellipses(len(dyn_names))
            _hide_extra_collision_ellipses(len(dyn_names))
            _hide_extra_apf_influence_ellipses(len(dyn_names))
        elif n > 0:
            ped_scatter.set_offsets(pos[:n])
            for i in range(n):
                vx, vy = spd[i]
                vmag   = math.sqrt(vx**2 + vy**2)
                theta  = ped_heading_deg(vx, vy)
                a, b = ped_ellipse_axes(vmag, i, deploy=ped_deploy)
                ell = _ensure_ellipse(i)
                ell.set_center((pos[i, 0], pos[i, 1]))
                ell.width = 2 * a
                ell.height = 2 * b
                ell.angle = theta
                ell.set_visible(True)

                ca, cb = ped_collision_ellipse_axes(vmag, i, deploy=ped_deploy)
                coll = _ensure_collision_ellipse(i)
                coll.set_center((pos[i, 0], pos[i, 1]))
                coll.width = 2 * ca
                coll.height = 2 * cb
                coll.angle = theta
                coll.set_visible(True)

                _sz, (_, _), (ia, ib), strength = ped_apf_model(vmag, i, deploy=ped_deploy)
                infl = _ensure_apf_influence_ellipse(i)
                infl.set_center((pos[i, 0], pos[i, 1]))
                infl.width = 2 * ia
                infl.height = 2 * ib
                infl.angle = theta
                infl.set_alpha(0.12 + 0.28 * strength)
                infl.set_linewidth(1.0 + 1.2 * strength)
                if i == 0:
                    infl.set_label('APF influence')
                infl.set_visible(True)
            _hide_extra_ellipses(n)
            _hide_extra_collision_ellipses(n)
            _hide_extra_apf_influence_ellipses(n)
        else:
            ped_scatter.set_offsets(np.empty((0, 2)))
            _hide_extra_ellipses(0)
            _hide_extra_collision_ellipses(0)
            _hide_extra_apf_influence_ellipses(0)

        # ── static / MoCap rectangles ──
        nonlocal mocap_status_text
        if not hasattr(update, '_mocap_draw_key'):
            update._mocap_draw_key = None
        if using_mocap:
            missing = [MOCAP_STATIC_NAMES[i - 1]
                       for i in range(1, MOCAP_OBSTACLE_COUNT + 1)
                       if i not in mocap_poses]
            mocap_draw_key = (
                tuple(sorted(mocap_poses.items())),
                tuple(sorted(dynamic_mocap_poses.items())),
                tuple(missing),
            )
        else:
            missing = []
            mocap_draw_key = ('sim', tuple(tuple(r) for r in display_rects))

        if mocap_draw_key != update._mocap_draw_key:
            update._mocap_draw_key = mocap_draw_key
            for r in rect_patches:
                r.remove()
            rect_patches.clear()
            for r in expanded_rect_patches:
                r.remove()
            expanded_rect_patches.clear()
            for t in mocap_label_artists:
                try:
                    t.remove()
                except Exception:
                    pass
            mocap_label_artists.clear()
            for a in mocap_center_artists:
                try:
                    a.remove()
                except Exception:
                    pass
            mocap_center_artists.clear()
            if mocap_status_text is not None:
                try:
                    mocap_status_text.remove()
                except Exception:
                    pass
                mocap_status_text = None

            if using_mocap:
                for idx in sorted(mocap_poses.keys()):
                    cx, cy = mocap_poses[idx]
                    w, h = (MOCAP_OBSTACLE_SIZES[idx - 1]
                            if idx - 1 < len(MOCAP_OBSTACLE_SIZES)
                            else MOCAP_OBSTACLE_SIZES[-1])
                    x, y, rw, rh = center_to_rect(cx, cy, w, h)
                    face, edge = MOCAP_OBSTACLE_COLORS.get(
                        idx, ('#b794f6', '#6b21a8'))
                    r = Rectangle(
                        (x, y), rw, rh, fill=True,
                        facecolor=face, alpha=0.5,
                        edgecolor=edge, linewidth=2.5,
                        label='MoCap obstacles' if idx == 1 else None,
                    )
                    ax.add_patch(r)
                    rect_patches.append(r)

                    dot, = ax.plot(
                        [cx], [cy], 'o', color=edge, markersize=9,
                        markeredgecolor='white', markeredgewidth=1.2, zorder=10)
                    mocap_center_artists.append(dot)

                    ang = math.radians(MOCAP_LABEL_ANGLES_DEG.get(idx, 90))
                    lx = cx + MOCAP_LABEL_OFFSET * math.cos(ang)
                    ly = cy + MOCAP_LABEL_OFFSET * math.sin(ang)
                    label = (MOCAP_STATIC_NAMES[idx - 1]
                             if idx - 1 < len(MOCAP_STATIC_NAMES) else str(idx))
                    txt = ax.text(
                        lx, ly, label, ha='center', va='center',
                        fontsize=10, fontweight='bold', color='white',
                        bbox=dict(boxstyle='round,pad=0.2', facecolor=edge,
                                  edgecolor='white', alpha=0.9),
                        zorder=11)
                    mocap_label_artists.append(txt)

                base_rects = poses_to_rects(mocap_poses)
                expanded, _ = sim_expanded_rects(base_rects, buffer=COLLISION_RECT_BUFFER)
                for j, (ex, ey, ew, eh) in enumerate(expanded):
                    r2 = Rectangle(
                        (ex, ey), ew, eh, fill=False,
                        edgecolor='#ea580c', linestyle='--', linewidth=1.5,
                        zorder=3,
                        label='Collision boundary' if j == 0 else None,
                    )
                    ax.add_patch(r2)
                    expanded_rect_patches.append(r2)

                status = f'Static: {len(mocap_poses)}/{MOCAP_OBSTACLE_COUNT}'
                if dynamic_mocap_poses:
                    status += f'  Dynamic: {len(dynamic_mocap_poses)}'
                if missing:
                    status += f'  (missing: {", ".join(missing)})'
                mocap_status_text = ax.text(
                    0.02, 0.02, status, transform=ax.transAxes,
                    fontsize=8, color='#374151', va='bottom',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                              edgecolor='#d1d5db', alpha=0.9),
                    zorder=12)
            else:
                for i, (x, y, w, h) in enumerate(display_rects):
                    r = Rectangle((x, y), w, h, fill=True,
                                   facecolor='orange', alpha=0.45,
                                   edgecolor='red', linewidth=2,
                                   label='Sim obstacles' if i == 0 else None)
                    ax.add_patch(r)
                    rect_patches.append(r)

                expanded = compute_expanded_rects(display_rects, buffer=COLLISION_RECT_BUFFER)
                for j, (ex, ey, ew, eh) in enumerate(expanded):
                    r2 = Rectangle(
                        (ex, ey), ew, eh, fill=False,
                        edgecolor='#ea580c', linestyle='--', linewidth=1.5,
                        zorder=3,
                        label='Collision boundary' if j == 0 else None,
                    )
                    ax.add_patch(r2)
                    expanded_rect_patches.append(r2)

        # ── robot paths (last PATH_TRAIL_POINTS only) ──
        if len(p_rrt) > 1 and not (using_mocap and _looks_like_sim_start(p_rrt[0])):
            arr = np.array(_tail_path(p_rrt))
            path_rrt_line.set_data(arr[:, 0], arr[:, 1])
        else:
            path_rrt_line.set_data([], [])
        if len(p_astar) > 1 and not (using_mocap and _looks_like_sim_start(p_astar[0])):
            arr = np.array(_tail_path(p_astar))
            path_astar_line.set_data(arr[:, 0], arr[:, 1])
        else:
            path_astar_line.set_data([], [])

        # ── robot dots ──
        show_rrt = not (using_mocap and _looks_like_sim_start(q_rrt))
        show_astar = not (using_mocap and _looks_like_sim_start(q_astar))
        robot_rrt_dot.set_data([q_rrt[0]] if show_rrt else [], [q_rrt[1]] if show_rrt else [])
        robot_astar_dot.set_data([q_astar[0]] if show_astar else [], [q_astar[1]] if show_astar else [])

        # ── goal markers ──
        show_g_rrt = not (using_mocap and _looks_like_sim_start(g_rrt))
        show_g_astar = not (using_mocap and _looks_like_sim_start(g_astar))
        goal_rrt_dot.set_data([g_rrt[0]] if show_g_rrt else [], [g_rrt[1]] if show_g_rrt else [])
        goal_astar_dot.set_data([g_astar[0]] if show_g_astar else [], [g_astar[1]] if show_g_astar else [])

        return (tsp_global_line, tsp_polyline_line, path_rrt_line, path_astar_line,
                robot_rrt_dot, robot_astar_dot,
                goal_rrt_dot, goal_astar_dot,
                ped_scatter)

    ani = animation.FuncAnimation(
        fig, update, interval=33, blit=False)  # 30 Hz — smoother animation

    plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()

    # spin ROS2 in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # matplotlib must run on main thread
    run_visualizer()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
