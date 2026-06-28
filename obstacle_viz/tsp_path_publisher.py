#!/usr/bin/env python3
"""
tsp_path_publisher.py
Runs the TSP planner once on startup, then publishes:
  /tsp_path       — LINE_STRIP of the full geometric path  (red)
  /tsp_waypoints  — SPHERE for each waypoint               (purple)
Both topics use transient-local QoS so RViz receives them even if it
starts after this node.

All TSP logic is copied verbatim from TSP_main.py so the two files stay
in sync.  Only the obstacle / waypoint definitions at the top need
editing to change the environment.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from nav_stack.mission.mission_config import get_mission

# ─────────────────────────────────────────────────────────────────────────────
# Environment  (must match obstacle_publisher.py)
# ─────────────────────────────────────────────────────────────────────────────
RECT_OBSTACLES = [
    [-40,   -30,  12, 3],
    [ 20,   -35,   6, 3],
    [-45,    15,  10, 4],
    [  5,    25,   3, 2],
    [ -6,  -6.5,  12, 3],
    [-13.5, -23.0,12, 6],
]

WAYPOINTS = get_mission('simulation')['waypoints']

Q_START = get_mission('simulation')['start']
Q_GOAL  = get_mission('simulation')['goal']

import math
import random

import numpy as np

# PSO hyper-parameters
PSO_RUNS       = 20
PSO_PARTICLES  = 250
PSO_ITER       = 5000

# ─────────────────────────────────────────────────────────────────────────────
# TSP geometry helpers  (verbatim from TSP_main.py)
# ─────────────────────────────────────────────────────────────────────────────

def rectangle_corners_center(rect):
    x, y, w, h = rect
    return [
        (x, y), (x+w, y), (x+w, y+h), (x, y+h),
        (x + w/2, y + h/2)
    ]

def line_mb(p1, p2):
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    if x2 == x1:
        return ("vertical",   round(x1, 2))
    if y2 == y1:
        return ("horizontal", round(y1, 2))
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    return ("normal", round(m, 2), round(b, 2))

def computeRectangleCircumcircle(rect):
    *_, c4 = rectangle_corners_center(rect)
    w, h = rect[2], rect[3]
    return c4, math.sqrt((w/2)**2 + (h/2)**2)

def eq_to_abc(eq):
    t = eq[0]
    if t == "normal":
        m, b = eq[1], eq[2]
        return (m, -1.0, -b)
    if t == "vertical":
        return (1.0, 0.0, eq[1])
    if t == "horizontal":
        return (0.0, 1.0, eq[1])
    raise ValueError(f"Unknown eq type: {eq}")

def point_to_line_distance(eq, p):
    a, b, c = eq_to_abc(eq)
    c = -c
    x0, y0 = p
    return abs(a*x0 + b*y0 + c) / math.sqrt(a*a + b*b)

def point_in_rect(pt, rect):
    X, Y = pt
    x, y, w, h = rect
    return (x <= X <= x + w) and (y <= Y <= y + h)

def calculateDistance(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def check_rectHit(line_store, rect_obstacles, expanded_rects):
    p2      = np.array(line_store["p2"])
    p2_goal = np.array(line_store["p2_goal"])
    eq      = line_store["equation"]
    true_hit_rects = []
    for rect, erect in zip(rect_obstacles, expanded_rects):
        eps = 0.015
        center, radius = computeRectangleCircumcircle(rect)
        if point_to_line_distance(eq, center) >= radius + eps:
            continue
        hit = False
        for t in np.linspace(0, 1, 50, endpoint=False)[1:]:
            p = p2 + t * (p2_goal - p2)
            if point_in_rect(p, erect):
                hit = True
                break
        if hit:
            true_hit_rects.append(rect)

    if true_hit_rects:
        return {**line_store, "hit_rects": true_hit_rects}
    return {**line_store, "hit_rects": 0}

def gotoCorner3(visited, myblue_line, goal,
                rect_obstacles, expanded_rects, eps_expanded_rects):
    myblue_line = check_rectHit(myblue_line, rect_obstacles, expanded_rects)
    p1        = myblue_line["p1"]
    p2        = myblue_line["p2"]
    p2_goal   = myblue_line["p2_goal"]
    p1_before = myblue_line["p1_before"]
    hit_rects = myblue_line["hit_rects"]
    valid_lines2 = []

    if hit_rects == 0:
        entry = {
            "p1_before": np.vstack([p1_before, p2, p2_goal]),
            "p1": p2, "p2": p2_goal, "p2_goal": p2_goal,
            "hit_rects": 0,
            "equation": line_mb(p2, p2_goal),
            "weight": myblue_line["weight"] + calculateDistance(p2, p2_goal),
        }
        valid_lines2.append(entry)
        goal.append(entry)
        return valid_lines2, goal

    if len(hit_rects) == 1:
        chosen_rect = hit_rects[0]
    else:
        dmin, chosen_rect = float("inf"), None
        for rect in hit_rects:
            c = np.array(rectangle_corners_center(rect)[4])
            d = np.linalg.norm(c - p2)
            if d < dmin:
                dmin, chosen_rect = d, rect

    idx              = rect_obstacles.index(chosen_rect)
    chosen_expanded  = expanded_rects[idx]
    chosen_expanded2 = eps_expanded_rects[idx]
    corners          = rectangle_corners_center(chosen_expanded)[:-1]
    candidate_lines  = []
    for corner in corners:
        candidate_lines.append({
            "p1": np.array(p2),
            "p2": np.array(corner),
            "p2_goal": p2_goal,
            "hit_rects": hit_rects,
            "weight": myblue_line["weight"] + calculateDistance(p2, corner),
            "p1_before": np.vstack([np.array(p1_before), np.array(p2)]),
            "equation": line_mb(corner, p2_goal),
        })

    for line in candidate_lines:
        enters = False
        p1l, p2l = np.array(line["p1"]), np.array(line["p2"])
        for t in np.linspace(0, 1, 50, endpoint=False)[1:]:
            if point_in_rect(p1l + t*(p2l - p1l), chosen_expanded2):
                enters = True
                break
        if not enters:
            valid_lines2.append(line)

    valid_lines2 = [l for l in valid_lines2
                    if not any(np.allclose(l["p2"], v) for v in visited)]

    visited_brother = visited.copy()
    valid_lines3 = []
    for line in valid_lines2:
        visited_brother.append(line["p2"])
        dummy = gotoCorner3(visited_brother, line, goal,
                            rect_obstacles, expanded_rects, eps_expanded_rects)
        if dummy:
            valid_lines3.append(dummy[0])

    return valid_lines3, goal

def bestPath(waypoints, q, q_goal, rect_obstacles, expanded_rects, eps_expanded_rects):
    nodes = [tuple(q)] + [tuple(w) for w in waypoints] + [tuple(q_goal)]
    N = len(nodes)
    traveller_sm = np.empty((N, N))
    route_sm     = np.empty((N, N), dtype=object)

    for i, p1 in enumerate(nodes):
        for j, p2 in enumerate(nodes):
            if i == j:
                continue
            eq = line_mb(p1, p2)
            line_store = {
                "p1": p1, "p2": p1, "equation": eq,
                "p1_before": p1, "weight": 0,
                "p2_goal": p2, "hit_rects": 0,
            }
            goal = []
            try:
                gotoCorner3([line_store["p2"]], line_store, goal,
                            rect_obstacles, expanded_rects, eps_expanded_rects)
            except Exception:
                pass

            if not goal:
                traveller_sm[i, j] = 1e9
                route_sm[i, j]     = np.array([p1, p2])
                continue

            weights    = [it["weight"] for it in goal]
            min_w      = min(weights)
            best_item  = next(it for it in goal if it["weight"] == min_w)
            traveller_sm[i, j] = min_w
            route_sm[i, j]     = best_item["p1_before"]

    return traveller_sm, route_sm

# ─────────────────────────────────────────────────────────────────────────────
# PSO  (verbatim from TSP_main.py)
# ─────────────────────────────────────────────────────────────────────────────

def build_tsp_indices(q, q_goal, waypoints):
    nodes = [tuple(q)] + [tuple(w) for w in waypoints] + [tuple(q_goal)]
    N = len(nodes)
    START, END = 0, N - 1
    WAYPOINTS  = [i for i in range(N) if i not in (START, END)]
    return N, START, END, WAYPOINTS, nodes

def random_path(START, END, WAYPOINTS):
    mid = WAYPOINTS[:]
    random.shuffle(mid)
    return [START] + mid + [END]

def fitness(path, traveller_sm):
    return sum(traveller_sm[a, b] for a, b in zip(path[:-1], path[1:]))

def get_swaps(p1, p2):
    swaps, temp = [], p1[:]
    for i in range(1, len(p1)-1):
        if temp[i] != p2[i]:
            j = temp.index(p2[i])
            swaps.append((i, j))
            temp[i], temp[j] = temp[j], temp[i]
    return swaps

def apply_swaps(path, swaps):
    new = path[:]
    for i, j in swaps:
        new[i], new[j] = new[j], new[i]
    return new

def PSO_TSP(traveller_sm, START, END, WAYPOINTS,
            n_particles=30, n_iter=200, w=0.4, c1=1.5, c2=1.5):
    particles  = [random_path(START, END, WAYPOINTS) for _ in range(n_particles)]
    pbest      = particles[:]
    pbest_cost = [fitness(p, traveller_sm) for p in particles]
    gbest_idx  = int(np.argmin(pbest_cost))
    gbest      = pbest[gbest_idx][:]
    gbest_cost = pbest_cost[gbest_idx]
    velocities = [[] for _ in range(n_particles)]

    for _ in range(n_iter):
        for i in range(n_particles):
            new_vel = []
            for s in velocities[i]:
                if random.random() < w:   new_vel.append(s)
            for s in get_swaps(particles[i], pbest[i]):
                if random.random() < c1:  new_vel.append(s)
            for s in get_swaps(particles[i], gbest):
                if random.random() < c2:  new_vel.append(s)
            velocities[i] = new_vel

            new_path = apply_swaps(particles[i], new_vel)
            particles[i] = new_path
            cost = fitness(new_path, traveller_sm)
            if cost < pbest_cost[i]:
                pbest[i], pbest_cost[i] = new_path, cost
                if cost < gbest_cost:
                    gbest, gbest_cost = new_path[:], cost

    return gbest, gbest_cost

def build_full_geometric_path(best_path, route_sm):
    full = []
    for a, b in zip(best_path[:-1], best_path[1:]):
        seg = route_sm[a, b]
        if seg is None or len(seg) == 0:
            continue
        full.extend(seg if not full else seg[2:])
    return np.array(full)

# ─────────────────────────────────────────────────────────────────────────────
# ROS2 node
# ─────────────────────────────────────────────────────────────────────────────

class TspPathPublisher(Node):
    def __init__(self):
        super().__init__('tsp_path_publisher')

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(Marker,      '/tsp_path',      latched)
        self.wp_pub   = self.create_publisher(MarkerArray, '/tsp_waypoints', latched)

        self.get_logger().info('Computing TSP path — this may take a minute …')
        path_pts = self._compute_tsp()
        self.get_logger().info(f'TSP done — {len(path_pts)} path points.')

        self._path_marker    = self._make_path_marker(path_pts)
        self._wp_markers     = self._make_waypoint_markers()

        # publish once immediately, then keep alive with a slow timer
        self._publish_all()
        self.create_timer(2.0, self._publish_all)

    # ── TSP computation ───────────────────────────────────────────────────────

    def _compute_tsp(self):
        expanded_rects = [
            [x-1, y-1, w+2, h+2] for x, y, w, h in RECT_OBSTACLES
        ]
        eps = 0.001
        eps_expanded = [
            [x-1+eps, y-1+eps, w+2-2*eps, h+2-2*eps]
            for x, y, w, h in RECT_OBSTACLES
        ]

        traveller_sm, route_sm = bestPath(
            WAYPOINTS, Q_START, Q_GOAL,
            RECT_OBSTACLES, expanded_rects, eps_expanded
        )
        N, START, END, WP_IDX, _ = build_tsp_indices(Q_START, Q_GOAL, WAYPOINTS)

        best_path, best_cost = None, float('inf')
        for run in range(PSO_RUNS):
            path, cost = PSO_TSP(
                traveller_sm, START=START, END=END, WAYPOINTS=WP_IDX,
                n_particles=PSO_PARTICLES, n_iter=PSO_ITER,
            )
            self.get_logger().info(f'  PSO run {run+1}/{PSO_RUNS}  cost={cost:.2f}')
            if cost < best_cost:
                best_cost, best_path = cost, path[:]

        self.get_logger().info(f'Best TSP cost: {best_cost:.2f}  path: {best_path}')
        return build_full_geometric_path(best_path, route_sm)

    # ── Marker builders ───────────────────────────────────────────────────────

    def _make_path_marker(self, pts):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns   = 'tsp_path'
        m.id   = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.4          # line width (metres)
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0
        m.pose.orientation.w = 1.0

        from geometry_msgs.msg import Point
        for pt in pts:
            p = Point()
            p.x, p.y, p.z = float(pt[0]), float(pt[1]), 0.1
            m.points.append(p)
        return m

    def _make_waypoint_markers(self):
        arr = MarkerArray()
        for i, wp in enumerate(WAYPOINTS):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns     = 'tsp_waypoints'
            m.id     = i
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wp[0])
            m.pose.position.y = float(wp[1])
            m.pose.position.z = 1.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 2.0
            m.color.r = 0.6
            m.color.g = 0.0
            m.color.b = 0.9
            m.color.a = 0.9
            arr.markers.append(m)
        return arr

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish_all(self):
        stamp = self.get_clock().now().to_msg()
        self._path_marker.header.stamp = stamp
        for m in self._wp_markers.markers:
            m.header.stamp = stamp
        self.path_pub.publish(self._path_marker)
        self.wp_pub.publish(self._wp_markers)


def main(args=None):
    rclpy.init(args=args)
    node = TspPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
