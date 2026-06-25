# ═══════════════════════════════════════════════════════════════════════════════
# trajectory_project — Makefile
# ═══════════════════════════════════════════════════════════════════════════════
#
# QUICK START
# ───────────────────────────────────────────────────
#
#   # 1. Start MoCap streaming (Terminal 1):
#   make mocap
#   # Or without OptiTrack hardware:
#   make fake-mocap                      # static poses from fake_mocap_poses.json
#
#   # 2. Echo obstacle poses (Terminal 2):
#   make mocap-echo OBS=obstacle1        # or obstacle2 / obstacle3
#
#   # 3. Full ROS2 simulation stack (one terminal):
#   make stop
#   make ros                             # wait ~90s for TSP, then plot opens
#   make stop                            # when done
#
#   # Full ROS2 simulation stack (three terminals):
#   make stop
#   make ped                             # Terminal 1
#   make planner                         # Terminal 2  (~90s TSP on startup)
#   make viz                             # Terminal 3  (after planner ready)
#   make stop                            # when done
#
#   # Offline matplotlib (no ROS):
#   make tsp-save                        # first time only
#   make mc
#
# Override defaults:  make ped N_PEDS=12 SPEED=1.5 HZ=25
#
# ═══════════════════════════════════════════════════════════════════════════════

.DEFAULT_GOAL := help

# ── ROS2 environment ───────────────────────────────────────────────────────────
ROS_SETUP    = source /opt/ros/jazzy/setup.bash
VRPN_SETUP   = source $(PROJECT_DIR)/vrpn_ws/install/setup.bash
RMW          = export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CLEAN_ENV    = export PYTHONPATH="" && export LD_LIBRARY_PATH=""
LD_PATH      = export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/home/egeozgul/Desktop/luci_ws/install/luci_messages/lib:/usr/local/cuda-12.8/lib64
ROS_ENV      = $(ROS_SETUP) && $(RMW) && $(LD_PATH)
MOCAP_ENV    = $(CLEAN_ENV) && $(ROS_SETUP) && $(VRPN_SETUP) && $(RMW)

# ── MoCap settings ─────────────────────────────────────────────────────────────
MOCAP_IP    ?= 192.168.0.2
MOCAP_PORT  ?= 3883
OBS         ?= obstacle1

# ── Paths & defaults ───────────────────────────────────────────────────────────
PROJECT_DIR  = $(shell pwd)
OBSTACLE_VIZ = $(PROJECT_DIR)/obstacle_viz
N_PEDS      ?= 8
SPEED       ?= 1.2
SPEED_DEPLOY ?= 0.87
HZ          ?= 25

.PHONY: help \
        mocap mocap-echo mocap-list fake-mocap \
        tsp tsp-save compute-tsp \
        mc mc-astar mc5 sim \
        ros ped ped-deploy planner viz rviz rviz-mocap \
        stop copy-deps

# ═══════════════════════════════════════════════════════════════════════════════
help:
	@echo ""
	@echo "trajectory_project — available targets"
	@echo "══════════════════════════════════════════════════════════════════════"
	@echo ""
	@echo "MOCAP (OptiTrack → ROS2 via VRPN)"
	@echo "  make mocap             Connect to MoCap PC and publish obstacle poses"
	@echo "                         (IP=$(MOCAP_IP)  port=$(MOCAP_PORT))"
	@echo "  make mocap-echo        Print live pose of one obstacle topic"
	@echo "                         Override: make mocap-echo OBS=obstacle2"
	@echo "  make mocap-list        List all active /vrpn_mocap/* topics"
	@echo "  make fake-mocap        Publish static fake /vrpn_mocap/*/pose (no OptiTrack)"
	@echo "                         Edit fake_mocap_poses.json (auto-reloaded every 5s)"
	@echo "  Frame: MoCap Y→map Z; ground plane MoCap X→map X, MoCap Z→map Y"
	@echo "  Missions: mission_waypoints.json (waypoints, viewport, obstacle pose names)"
	@echo ""
	@echo "OFFLINE (matplotlib, no ROS required)"
	@echo "  make tsp-save          Compute TSP path → tsp_path.npy + tsp_polyline.npy"
	@echo "  make tsp               Plot TSP path only (no save)"
	@echo "  make mc                Monte Carlo: RRT vs A* comparison (needs tsp_path.npy)"
	@echo "  make mc-astar          Monte Carlo: A* only variant"
	@echo "  make mc5               Monte Carlo: main5_MCsim variant"
	@echo ""
	@echo "ROS2 LIVE STACK"
	@echo "  make ros               Launch pedestrian sim + planner + visualizer"
	@echo "  make ped               Pedestrian simulator only  [N_PEDS=$(N_PEDS) SPEED=$(SPEED) HZ=$(HZ)]"
	@echo "  make ped-deploy        Pedestrian sim in deployment viewport (6m x 6m)"
	@echo "                         [N_PEDS=$(N_PEDS) SPEED=$(SPEED_DEPLOY) HZ=$(HZ)]"
	@echo "  make planner           Planner node only (RRT + A*, waits for /pedestrian_state)"
	@echo "  make viz               Matplotlib visualizer only"
	@echo "  make rviz              RViz: static sim obstacles + TSP path preview"
	@echo "  make rviz-mocap        RViz: live MoCap obstacles (run make mocap or fake-mocap first)"
	@echo "  make stop              Kill all running nodes"
	@echo ""
	@echo "OTHER"
	@echo "  make copy-deps         Copy .py files from ~/Downloads/path_planning-main"
	@echo ""
	@echo "══════════════════════════════════════════════════════════════════════"
	@echo "Pedestrian controls (make ped / ped-deploy):  h/j add/remove  u/t speed  p status  q quit"
	@echo "Planner controls   (make planner): q quit"
	@echo ""

# ═══════════════════════════════════════════════════════════════════════════════
# MOCAP — OptiTrack → ROS2
# ═══════════════════════════════════════════════════════════════════════════════

mocap:
	@echo "Connecting to MoCap PC at $(MOCAP_IP):$(MOCAP_PORT) ..."
	@echo "Publishing poses on /vrpn_mocap/obstacle1/pose  obstacle2  obstacle3"
	@bash -c "$(MOCAP_ENV) && ros2 launch vrpn_mocap client.launch.yaml server:=$(MOCAP_IP) port:=$(MOCAP_PORT)"

mocap-echo:
	@echo "Echoing /vrpn_mocap/$(OBS)/pose  (Ctrl+C to stop)"
	@bash -c "$(MOCAP_ENV) && ros2 topic echo /vrpn_mocap/$(OBS)/pose"

mocap-list:
	@echo "Active MoCap topics:"
	@bash -c "$(MOCAP_ENV) && ros2 topic list | grep vrpn_mocap"

# ═══════════════════════════════════════════════════════════════════════════════
# OFFLINE — TSP & Monte Carlo
# ═══════════════════════════════════════════════════════════════════════════════

tsp:
	@echo "Running TSP planner (plot only, no save)..."
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && python3 TSP_main.py"

tsp-save: compute-tsp

compute-tsp:
	@echo "Computing TSP path and saving .npy files..."
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && python3 computeTSP.py"
	@test -f $(PROJECT_DIR)/tsp_path.npy && test -f $(PROJECT_DIR)/tsp_polyline.npy \
		|| (echo "ERROR: tsp_path.npy / tsp_polyline.npy were not created." && exit 1)
	@echo "Saved: tsp_path.npy  tsp_polyline.npy"

mc:
	@test -f $(PROJECT_DIR)/tsp_path.npy \
		|| (echo "ERROR: tsp_path.npy not found. Run 'make tsp-save' first." && exit 1)
	@echo "Running Monte Carlo simulation (RRT vs A*)..."
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && python3 main_sim_TSP_PF_RRT+Astar.py"

mc-astar:
	@test -f $(PROJECT_DIR)/tsp_path.npy \
		|| (echo "ERROR: tsp_path.npy not found. Run 'make tsp-save' first." && exit 1)
	@echo "Running Monte Carlo simulation (A* only)..."
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && python3 main_MCsim_TSP_PF_Astar.py"

mc5:
	@test -f $(PROJECT_DIR)/tsp_path.npy \
		|| (echo "ERROR: tsp_path.npy not found. Run 'make tsp-save' first." && exit 1)
	@echo "Running Monte Carlo simulation (main5_MCsim)..."
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && python3 main5_MCsim.py"

sim: mc

# ═══════════════════════════════════════════════════════════════════════════════
# ROS2 — live stack
# ═══════════════════════════════════════════════════════════════════════════════

ros:
	@echo "Launching full ROS2 stack (pedestrian + planner + visualizer)..."
	@echo "Planner TSP takes ~90s before the visualizer opens."
	@bash $(PROJECT_DIR)/scripts/run_ros2_planner.sh

ped:
	@echo "Starting pedestrian simulator  (N=$(N_PEDS)  speed=$(SPEED) m/s  hz=$(HZ))"
	@echo "Controls: + add  - remove  ] faster  [ slower  p status  q quit"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' python3 pedestrian_sim_node.py --n_pedestrians $(N_PEDS) --speed $(SPEED) --hz $(HZ)"


ped-deploy:
	@echo "Starting pedestrian simulator in deployment mode (6m x 6m viewport)"
	@echo "  N=$(N_PEDS)  speed=$(SPEED_DEPLOY) m/s  hz=$(HZ)"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' python3 pedestrian_sim_node.py --n_pedestrians $(N_PEDS) --speed $(SPEED_DEPLOY) --hz $(HZ) --deploy"
planner:
	@echo "Starting planner node (RRT + A*)..."
	@echo "Requires pedestrian_sim_node publishing on /pedestrian_state."
	@echo "TSP computation may take ~90s on startup."
	@echo "Controls:  q quit"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 python3 -u planner_node.py"

viz:
	@echo "Starting matplotlib visualizer..."
	@echo "Requires pedestrian_sim_node and planner_node to be running."
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' python3 visualizer_node.py"

rviz:
	@echo "Launching RViz obstacle + TSP path preview..."
	@bash $(OBSTACLE_VIZ)/run.sh

rviz-mocap:
	@echo "Launching RViz with live MoCap obstacles..."
	@echo "Ensure 'make mocap' is running in another terminal."
	@bash $(OBSTACLE_VIZ)/run_mocap.sh

# ═══════════════════════════════════════════════════════════════════════════════
# STOP & UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

stop:
	@echo "Stopping all nodes..."
	@pkill -f pedestrian_sim_node.py  2>/dev/null || true
	@pkill -f planner_node.py         2>/dev/null || true
	@pkill -f visualizer_node.py      2>/dev/null || true
	@pkill -f mocap_obstacle_publisher.py 2>/dev/null || true
	@pkill -f obstacle_publisher.py   2>/dev/null || true
	@pkill -f tsp_path_publisher.py   2>/dev/null || true
	@pkill -f run_ros2_planner.sh     2>/dev/null || true
	@pkill -f rviz2                   2>/dev/null || true
	@pkill -f vrpn_mocap              2>/dev/null || true
	@pkill -f fake_mocap_node.py      2>/dev/null || true
	@echo "Done."

copy-deps:
	@test -d $(HOME)/Downloads/path_planning-main \
		|| (echo "ERROR: path_planning-main not found in ~/Downloads. Unzip it first." && exit 1)
	@echo "Copying dependency files..."
	@cp $(HOME)/Downloads/path_planning-main/tsp_pf_rrt+astar_combined/*.py $(PROJECT_DIR)/
	@echo "Done. Files in $(PROJECT_DIR):"
	@ls $(PROJECT_DIR)/*.py

fake-mocap:
	@echo "Starting fake MoCap node (last known lab positions)..."
	@echo "Edit fake_mocap_poses.json to change positions (auto-reloaded every 5s)"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' python3 fake_mocap_node.py"
