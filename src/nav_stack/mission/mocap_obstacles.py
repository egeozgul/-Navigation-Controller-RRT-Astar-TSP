"""
MoCap static obstacles for TSP / global path planning.

Coordinate transform (OptiTrack / VRPN ``world`` → ROS ``map`` / RViz):

  MoCap frame (Y-up) → ROS map (Z-up):

    MoCap Y  →  map Z   (vertical / up — “Y becomes Z”)
    MoCap X  →  map X   (ground plane)
    MoCap Z  →  map Y   (ground plane)

  2D ground plane (planner + matplotlib visualizer):
    map_x =  mocap_x   (MOCAP_X_SIGN, default +1)
    map_y =  mocap_z   (MOCAP_Z_SIGN, default +1)
    (mocap_y is height → map_z, not used in 2D)

  3D height (RViz cubes):
    map_z =  mocap_y   (MOCAP_Y_SIGN, default +1)

Override any axis sign: MOCAP_X_SIGN, MOCAP_Y_SIGN, MOCAP_Z_SIGN (±1).
Optional translation after rotation: MOCAP_OFFSET_X/Y/Z (metres, map frame).
"""
import os
from typing import Dict, List, Tuple

from nav_stack.mission.mission_config import get_mission

_deploy = get_mission('deployment')
MOCAP_POSE_TOPICS = _deploy['static_pose_topics']
MOCAP_STATIC_NAMES = _deploy['static_obstacle_names']
MOCAP_DYNAMIC_NAMES = _deploy['dynamic_obstacle_names']
MOCAP_DYNAMIC_POSE_TOPICS = _deploy['dynamic_pose_topics']
MOCAP_POSE_NAMESPACE = _deploy['pose_namespace']

_deploy_sizes = _deploy.get('static_obstacle_sizes', [])
MOCAP_OBSTACLE_SIZES = tuple(
    tuple(s) for s in _deploy_sizes
) if _deploy_sizes else (
    (0.3, 0.3),
    (0.3, 0.3),
    (0.3, 0.3),
    (0.3, 0.3),
    (0.3, 0.3),
)

MOCAP_OBSTACLE_COLORS = {
    1: ('#7c3aed', '#5b21b6'),
    2: ('#0891b2', '#0e7490'),
    3: ('#d97706', '#b45309'),
    4: ('#16a34a', '#15803d'),
    5: ('#dc2626', '#b91c1c'),
}

MOCAP_LABEL_ANGLES_DEG = {1: 90, 2: 210, 3: 330}
MOCAP_LABEL_OFFSET = 0.65
MOCAP_OBSTACLE_COUNT = len(MOCAP_STATIC_NAMES)

MOCAP_WAIT_SEC = float(os.environ.get('MOCAP_WAIT_SEC', '5.0'))

# Translation in map frame, applied after rotation (lab calibration).
MOCAP_MAP_OFFSET = (
    float(os.environ.get('MOCAP_OFFSET_X', '0.0')),
    float(os.environ.get('MOCAP_OFFSET_Y', '0.0')),
    float(os.environ.get('MOCAP_OFFSET_Z', '0.0')),
)

# Per-axis sign (±1). Standard Y-up → Z-up: mocap (x,z) → map (x,y), mocap y → map z.
MOCAP_X_SIGN = float(os.environ.get('MOCAP_X_SIGN', '1.0'))
MOCAP_Y_SIGN = float(os.environ.get('MOCAP_Y_SIGN', '1.0'))
MOCAP_Z_SIGN = float(os.environ.get('MOCAP_Z_SIGN', '1.0'))


def mocap_position_to_map(mx: float, my: float, mz: float) -> Tuple[float, float, float]:
    """
    OptiTrack (Y-up) position → ROS map (Z-up).

    map_x =  MOCAP_X_SIGN * mx + offset_x   (MoCap X → map X)
    map_y =  MOCAP_Z_SIGN * mz + offset_y   (MoCap Z → map Y)
    map_z =  MOCAP_Y_SIGN * my + offset_z   (MoCap Y → map Z)
    """
    ox, oy, oz = MOCAP_MAP_OFFSET
    return (
        MOCAP_Z_SIGN * mz + ox,
        MOCAP_X_SIGN * mx + oy,
        MOCAP_Y_SIGN * my + oz,
    )


def pose_xy(msg) -> Tuple[float, float]:
    """PoseStamped in VRPN ``world`` → (map_x, map_y) for 2D planning/viz."""
    p = msg.pose.position
    map_x, map_y, _ = mocap_position_to_map(
        float(p.x), float(p.y), float(p.z))
    return map_x, map_y


def center_to_rect(cx: float, cy: float, width: float, height: float) -> List[float]:
    return [cx - width / 2.0, cy - height / 2.0, width, height]


def poses_to_rects(
    poses: Dict[int, Tuple[float, float]],
    sizes=MOCAP_OBSTACLE_SIZES,
) -> List[List[float]]:
    rects = []
    for idx in sorted(poses.keys()):
        cx, cy = poses[idx]
        w, h = sizes[idx - 1] if idx - 1 < len(sizes) else sizes[-1]
        rects.append(center_to_rect(cx, cy, w, h))
    return rects


def sim_expanded_rects(rect_obstacles, buffer: float = 0.1, eps: float = 0.001):
    expanded = []
    eps_expanded = []
    for x, y, w, h in rect_obstacles:
        expanded.append([x - buffer, y - buffer, w + 2 * buffer, h + 2 * buffer])
        eps_expanded.append([
            x - buffer + eps,
            y - buffer + eps,
            w + 2 * buffer - 2 * eps,
            h + 2 * buffer - 2 * eps,
        ])
    return expanded, eps_expanded


def mocap_rects_from_poses(poses: Dict[int, Tuple[float, float]]):
    rects = poses_to_rects(poses)
    expanded, eps_expanded = sim_expanded_rects(rects)
    return rects, expanded, eps_expanded


def all_mocap_poses_received(
    poses: Dict[int, Tuple[float, float]],
    n: int | None = None,
) -> bool:
    n = n if n is not None else MOCAP_OBSTACLE_COUNT
    if n <= 0:
        return False
    return all(i in poses for i in range(1, n + 1))
