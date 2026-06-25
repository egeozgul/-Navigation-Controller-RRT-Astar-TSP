# trajectory_project

ROS2 stack for TSP global planning, local RRT + A* navigation, APF pedestrian avoidance, and live matplotlib visualization. Supports simulation and deployment (6 m × 6 m) with optional OptiTrack MoCap.

Run `make help` for all targets.

## Quick start

```bash
# Full ROS2 stack (one terminal)
make stop && make ros

# Or step by step
make ped          # Terminal 1 — pedestrians
make planner      # Terminal 2 — planner (~90s TSP on startup)
make viz          # Terminal 3 — visualizer

# Deployment viewport + fake MoCap (no hardware)
make fake-mocap   # Terminal 1
make ped-deploy   # Terminal 2
make planner      # Terminal 3
make viz          # Terminal 4
```

## Directory layout

```
trajectory_project/
├── Makefile                  # All make targets
├── mission_waypoints.json    # Sim + deployment waypoints, obstacles, viewport
├── fake_mocap_poses.json     # Static fake MoCap poses (make fake-mocap)
├── tsp_path.npy              # Cached TSP path (make tsp-save)
├── tsp_polyline.npy

├── planner_node.py           # ROS2 planner (RRT + A*, TSP, MoCap)
├── visualizer_node.py        # Matplotlib live viz
├── pedestrian_sim_node.py    # Pedestrian simulator
├── fake_mocap_node.py        # Fake /vrpn_mocap/*/pose publisher

├── APF_RRT_Astar.py          # APF, environment init, polyline helpers
├── rrt_planner_main.py       # RRT local planner
├── damla_Astar.py            # A* local planner
├── TSP_main.py               # TSP (offline plot)
├── TSP_Rachel.py             # TSP geometry / routing
├── computeTSP.py             # TSP → .npy files
├── mission_config.py         # Load mission_waypoints.json
├── mocap_obstacles.py        # MoCap obstacle rects / transforms
├── sim_reference_params.py   # Shared speeds, ellipse params
├── planner_ema_filter.py     # Optional EMA filter state

├── run_simulation.py         # Offline sim loop
├── main_sim_TSP_PF_RRT+Astar.py   # make mc
├── main_MCsim_TSP_PF_Astar.py     # make mc-astar
├── main5_MCsim.py                 # make mc5
├── APF5.py / APF_Astar.py         # Legacy APF variants (offline only)

├── scripts/
│   └── run_ros2_planner.sh   # make ros launcher
├── obstacle_viz/               # RViz static + MoCap obstacle publishers
├── vrpn_ws/                    # VRPN MoCap ROS2 package (make mocap)
├── results/                    # Offline sim PNG outputs
├── archive/
│   └── path_planning-main/     # Original reference codebase (read-only)
├── vendor/
│   └── luci-ros2-control/      # Unrelated vendored repo
└── docs/
    └── notes                   # Project notes
```

## Configuration

- **Missions:** edit `mission_waypoints.json` (`simulation` and `deployment` sections).
- **Fake MoCap:** edit `fake_mocap_poses.json` (auto-reloaded every 5 s).
- **Pedestrians (deploy):** `deployment.pedestrians` in mission JSON; override speed with `make ped-deploy SPEED_DEPLOY=0.87`.

## Requirements

- ROS 2 Jazzy (`/opt/ros/jazzy`)
- Python 3 with numpy, matplotlib
- VRPN workspace built under `vrpn_ws/` for `make mocap`
