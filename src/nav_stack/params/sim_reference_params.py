"""
Shared simulation parameters from path_planning-main/tsp_pf_rrt+astar_combined
(main_sim_TSP_PF_RRT+Astar.py + APF_RRT_Astar.py update loop).
"""
import numpy as np

from nav_stack.mission.mission_config import get_mission

_sim_mission = get_mission('simulation')

# Robot (reference: v_robot=30, dt=0.01 per animation frame @ 25 Hz)
V_ROBOT = 30.0
DT_SIM = 0.01
TOLERANCE = 1.0

# Deployment overrides (real robot in 6m x 6m space)
V_ROBOT_DEPLOY = 0.3      # m/s — real robot speed
DT_SIM_DEPLOY  = 0.04     # matches 25 Hz timer
TOLERANCE_DEPLOY = 0.35   # 35cm goal tolerance
Q_START = _sim_mission['start']
Q_GOAL_FINAL = _sim_mission['goal']

# Pedestrian speed scale (reference: base unit vectors * 80)
SPEED_SCALE = 80.0
VMAG_MIN = 5.0
VMAG_MAX = 24.5

# Ellipse body model (reference APF_RRT_Astar update / repulsive_force)
PED_A0 = 2.0
PED_B0 = 1.0
PED_ALPHA = 0.2
PED_BETA = 0.1
PED_A_MAX = 3.0
PED_B_MAX = 1.5

# Deployment-scaled pedestrian ellipse sizes (6m x 6m space)
PED_A0_DEPLOY    = 0.3
PED_B0_DEPLOY    = 0.2
PED_ALPHA_DEPLOY = 0.05
PED_BETA_DEPLOY  = 0.03
PED_A_MAX_DEPLOY = 0.5
PED_B_MAX_DEPLOY = 0.35

# APF repulsive_force pedestrian model (APF_RRT_Astar.repulsive_force)
APF_PED_A0_DEPLOY = 0.4
APF_PED_B0_DEPLOY = 0.3
APF_PED_ALPHA_DEPLOY = 0.1
APF_PED_BETA_DEPLOY = 0.05
APF_PED_A_MAX_DEPLOY = 0.8
APF_PED_B_MAX_DEPLOY = 0.6
APF_PED_D0_DEPLOY = 0.8

APF_PED_A0_SIM = 3.4
APF_PED_B0_SIM = 1.7
APF_PED_ALPHA_SIM = 0.2
APF_PED_BETA_SIM = 0.1
APF_PED_A_MAX_SIM = 5.1
APF_PED_B_MAX_SIM = 2.55
APF_PED_D0_SIM = 1.0

# Kalabalik platform — 24 pedestrians (main_sim_TSP_PF_RRT+Astar.py)
PED_INIT_POSITIONS = np.array([
    [-18.0, -10.0], [18, -20], [18, 8], [22, 26], [23, 15], [-23, 15],
    [5, 5], [-40, -30], [15, -10], [10, 5], [-30, 0], [-20, -20],
    [0, 0], [-35, 3], [-26, -27], [2, -10], [11, -12], [-3, -3],
    [-45, 0], [-25, -25], [8, 10], [-35, 13], [-21, -17], [21, -10],
], dtype=float)

_PED_SPEED_UNIT = np.array([
    [-0.1, 0.1], [-0.2, 0.2], [0.1, 0.2], [-0.2, -0.1], [0.1, -0.1],
    [-0.1, 0.1], [0.2, 0.1], [-0.1, 0.1], [-0.2, 0.1], [-0.2, 0.2],
    [-0.2, 0.2], [-0.2, 0.2], [-0.2, 0.2], [0.2, 0.1], [-0.1, 0.1],
    [-0.2, 0.1], [-0.1, 0.1], [-0.2, 0.2], [0.1, 0.2], [-0.2, -0.1],
    [0.1, -0.1], [-0.1, 0.1], [0.2, 0.1], [-0.1, 0.1],
], dtype=float)
PED_INIT_SPEEDS = _PED_SPEED_UNIT * SPEED_SCALE

PED_SIZES = np.array([
    1.2, 1.5, 1.0, 1.3, 0.8, 1.6, 1.1, 1.0,
    1.2, 1.8, 2.0, 1.8, 1.9, 1.4, 1.5, 1.8,
    1.2, 1.5, 1.0, 1.3, 0.8, 1.6, 1.1, 1.0,
], dtype=float)

DEFAULT_N_PEDS = len(PED_INIT_POSITIONS)
MAX_PEDS = 60


def ped_size_factor(index: int) -> float:
    return float(PED_SIZES[index % len(PED_SIZES)])


def ped_ellipse_axes(vmag: float, index: int, deploy: bool = False):
    """Return semi-axes (a, b) for pedestrian index at speed magnitude vmag."""
    if deploy:
        # Fixed size for all pedestrians in deployment mode
        return PED_A0_DEPLOY, PED_B0_DEPLOY
    s = ped_size_factor(index)
    a = min(PED_A0 * s + PED_ALPHA * vmag, PED_A_MAX)
    b = min(PED_B0 * s + PED_BETA  * vmag, PED_B_MAX)
    return a, b


def ped_collision_ellipse_axes(vmag: float, index: int, deploy: bool = False):
    """Semi-axes for RRT hard-collision ellipse (major axis along velocity)."""
    if deploy:
        return float(APF_PED_A0_DEPLOY), float(APF_PED_B0_DEPLOY)
    s = ped_size_factor(index)
    a = min(PED_A0 * s + PED_ALPHA * vmag, PED_A_MAX)
    b = min(PED_B0 * s + PED_BETA * vmag, PED_B_MAX)
    return float(a), float(b)


def ped_heading_from_speed(vx: float, vy: float, default: float = 0.0) -> float:
    """Heading (rad) for ellipse major axis; default when stationary."""
    if vx * vx + vy * vy < 1e-12:
        return default
    return float(np.arctan2(vy, vx))


def ped_heading_deg(vx: float, vy: float, default: float = 0.0) -> float:
    return float(np.degrees(ped_heading_from_speed(vx, vy, default)))


def _ped_ellipse_local_frame(q, obs, a: float, b: float, vx: float, vy: float):
    """Return velocity-aligned local coords and normalized ellipse radii."""
    dx = float(q[0] - obs[0])
    dy = float(q[1] - obs[1])
    theta = ped_heading_from_speed(vx, vy)
    c, s = np.cos(theta), np.sin(theta)
    local_major = dx * c + dy * s
    local_minor = -dx * s + dy * c
    eps = 1e-12
    u = local_major / (a + eps)
    v = local_minor / (b + eps)
    r = float(np.sqrt(u * u + v * v))
    return dx, dy, c, s, u, v, r, a, b, eps


def ped_ellipse_dE(q, obs, a: float, b: float, vx: float, vy: float) -> float:
    """
    Normalized ellipse distance in the pedestrian velocity frame.
    0 on the boundary, <0 inside, >0 outside.
    """
    *_, u, v, r, _, _, _ = _ped_ellipse_local_frame(q, obs, a, b, vx, vy)
    return r - 1.0


def ped_ellipse_dE_gradient(q, obs, a: float, b: float, vx: float, vy: float) -> np.ndarray:
    """Unit gradient of ellipse distance (repulsion direction for APF)."""
    dx, dy, c, s, u, v, r, a_ax, b_ax, eps = _ped_ellipse_local_frame(
        q, obs, a, b, vx, vy,
    )
    if r < 1e-9:
        n = float(np.hypot(dx, dy))
        if n < 1e-9:
            return np.array([1.0, 0.0])
        return np.array([dx / n, dy / n])
    g_major = u / ((a_ax + eps) * r)
    g_minor = v / ((b_ax + eps) * r)
    gx = g_major * c - g_minor * s
    gy = g_major * s + g_minor * c
    norm = float(np.hypot(gx, gy)) + 1e-9
    return np.array([gx / norm, gy / norm])


def ped_apf_size_factor_from_body(a_body: float, b_body: float, deploy: bool = False) -> float:
    """Scale APF ellipse from displayed body semi-axes (1.0 = nominal pedestrian)."""
    if deploy:
        ref_a, ref_b = PED_A0_DEPLOY, PED_B0_DEPLOY
    else:
        ref_a, ref_b = PED_A0, PED_B0
    return float(0.5 * (a_body / ref_a + b_body / ref_b))


def ped_apf_ellipse_axes(
    vmag: float,
    deploy: bool = False,
    size_factor: float = 1.0,
):
    """Core ellipse semi-axes used by repulsive_force (before d0 scaling)."""
    if deploy:
        a0, b0 = APF_PED_A0_DEPLOY, APF_PED_B0_DEPLOY
        alpha, beta = APF_PED_ALPHA_DEPLOY, APF_PED_BETA_DEPLOY
        a_max, b_max = APF_PED_A_MAX_DEPLOY, APF_PED_B_MAX_DEPLOY
    else:
        a0, b0 = APF_PED_A0_SIM, APF_PED_B0_SIM
        alpha, beta = APF_PED_ALPHA_SIM, APF_PED_BETA_SIM
        a_max, b_max = APF_PED_A_MAX_SIM, APF_PED_B_MAX_SIM
    s = float(size_factor)
    a = min(a0 * s + alpha * vmag, a_max)
    b = min(b0 * s + beta * vmag, b_max)
    return float(a), float(b)


def ped_apf_field_strength(
    vmag: float,
    deploy: bool = False,
    size_factor: float = 1.0,
) -> float:
    """Relative APF strength for visualization shading (size + speed only)."""
    s_max = float(np.max(PED_SIZES))
    size_term = min(float(size_factor) / s_max, 1.0) if s_max > 0 else 0.5
    vmax = 1.5 if deploy else 24.0
    speed_term = min(float(vmag) / vmax, 1.0)
    return float(np.clip(0.35 + 0.35 * size_term + 0.30 * speed_term, 0.35, 1.0))


def ped_apf_influence_axes(
    vmag: float,
    deploy: bool = False,
    size_factor: float = 1.0,
):
    """Semi-axes of the APF influence boundary where dE = d0."""
    a, b = ped_apf_ellipse_axes(vmag, deploy=deploy, size_factor=size_factor)
    d0 = APF_PED_D0_DEPLOY if deploy else APF_PED_D0_SIM
    scale = 1.0 + d0
    return a * scale, b * scale


def ped_apf_model(vmag: float, index: int, deploy: bool = False):
    """Body size factor + APF ellipse/influence axes for pedestrian index."""
    a_body, b_body = ped_ellipse_axes(vmag, index, deploy=deploy)
    size_f = ped_apf_size_factor_from_body(a_body, b_body, deploy=deploy)
    core = ped_apf_ellipse_axes(vmag, deploy=deploy, size_factor=size_f)
    influence = ped_apf_influence_axes(vmag, deploy=deploy, size_factor=size_f)
    strength = ped_apf_field_strength(vmag, deploy=deploy, size_factor=size_f)
    return size_f, core, influence, strength
