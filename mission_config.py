"""
Mission visit positions for TSP planning (simulation vs real-world deployment).

Loaded from mission_waypoints.json. When MoCap is active the planner uses
``deployment``; otherwise ``simulation``.

Deployment obstacle names are VRPN / MoCap rigid-body names. Full pose topics
are built as ``{namespace}/{name}/pose`` (default namespace ``/vrpn_mocap``).
"""
import json
import os
from typing import Dict, List, Literal, Tuple

import numpy as np

MissionMode = Literal['simulation', 'deployment']

DEFAULT_MISSION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'mission_waypoints.json',
)

DEFAULT_POSE_NAMESPACE = os.environ.get('MISSION_POSE_NAMESPACE', '/vrpn_mocap')


def _mission_file_path() -> str:
    return os.environ.get('MISSION_WAYPOINTS_JSON', DEFAULT_MISSION_FILE)


def pose_topics_from_names(
    names: List[str],
    namespace: str | None = None,
) -> Tuple[str, ...]:
    """Build ROS pose topic paths from rigid-body / obstacle names."""
    ns = (namespace or DEFAULT_POSE_NAMESPACE).rstrip('/')
    return tuple(f'{ns}/{name}/pose' for name in names)


def load_mission_file(path: str | None = None) -> Dict:
    path = path or _mission_file_path()
    with open(path, encoding='utf-8') as f:
        data = json.load(f)
    for mode in ('simulation', 'deployment'):
        if mode not in data:
            raise KeyError(f'mission_waypoints.json missing "{mode}" section')
        _validate_mission_section(data[mode], mode)
    return data


def _validate_mission_section(section: Dict, mode: str) -> None:
    for key in ('start', 'goal', 'waypoints'):
        if key not in section:
            raise KeyError(f'{mode}: missing "{key}"')
    if len(section['start']) != 2 or len(section['goal']) != 2:
        raise ValueError(f'{mode}: start and goal must be [x, y]')
    for i, wp in enumerate(section['waypoints']):
        if len(wp) != 2:
            raise ValueError(f'{mode}: waypoint {i} must be [x, y]')
    if 'viewport' in section:
        vp = section['viewport']
        for key in ('x_min', 'x_max', 'y_min', 'y_max'):
            if key not in vp:
                raise KeyError(f'{mode}.viewport: missing "{key}"')
        if vp['x_min'] >= vp['x_max'] or vp['y_min'] >= vp['y_max']:
            raise ValueError(f'{mode}.viewport: min must be less than max')
    if 'static_obstacle_sizes' in section:
        sizes = section['static_obstacle_sizes']
        if not isinstance(sizes, list) or not all(len(s) == 2 and all(v > 0 for v in s) for s in sizes):
            raise ValueError(f'{mode}.static_obstacle_sizes must be a list of [width, height] pairs with positive values')
    for key in ('static_obstacle_names', 'dynamic_obstacle_names'):
        if key not in section:
            raise KeyError(f'{mode}: missing "{key}"')
        names = section[key]
        if not isinstance(names, list) or not all(isinstance(n, str) for n in names):
            raise ValueError(f'{mode}.{key} must be a list of strings')


def _viewport_from_section(section: Dict) -> tuple[float, float, float, float]:
    if 'viewport' in section:
        vp = section['viewport']
        return float(vp['x_min']), float(vp['x_max']), float(vp['y_min']), float(vp['y_max'])
    pts = [section['start'], section['goal'], *section['waypoints']]
    arr = np.array(pts, dtype=float)
    pad = 5.0
    return (
        float(arr[:, 0].min() - pad),
        float(arr[:, 0].max() + pad),
        float(arr[:, 1].min() - pad),
        float(arr[:, 1].max() + pad),
    )


def get_mission(mode: MissionMode, path: str | None = None) -> Dict:
    """Return mission data for simulation or deployment."""
    section = load_mission_file(path)[mode]
    x_min, x_max, y_min, y_max = _viewport_from_section(section)
    namespace = section.get('pose_namespace', DEFAULT_POSE_NAMESPACE)
    static_names = list(section['static_obstacle_names'])
    dynamic_names = list(section['dynamic_obstacle_names'])
    return {
        'start': np.array(section['start'], dtype=float),
        'goal': np.array(section['goal'], dtype=float),
        'waypoints': np.array(section['waypoints'], dtype=float).reshape(-1, 2)
        if section['waypoints'] else np.empty((0, 2), dtype=float),
        'viewport': (x_min, x_max, y_min, y_max),
        'static_obstacle_names': static_names,
        'dynamic_obstacle_names': dynamic_names,
        'static_pose_topics': pose_topics_from_names(static_names, namespace),
        'dynamic_pose_topics': pose_topics_from_names(dynamic_names, namespace),
        'pose_namespace': namespace,
        'static_obstacle_sizes': [
            tuple(s) for s in section.get('static_obstacle_sizes', [])
        ],
        'waypoint_tolerance': float(section.get('waypoint_tolerance', 0.25)),
        'pedestrians': section.get('pedestrians', {'count': 8, 'speed': 1.2, 'hz': 25.0}),
    }


def mission_for_mocap(mocap_active: bool, path: str | None = None) -> Dict:
    mode: MissionMode = 'deployment' if mocap_active else 'simulation'
    return get_mission(mode, path=path)
