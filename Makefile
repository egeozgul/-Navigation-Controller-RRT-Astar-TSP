# trajectory_project — Makefile
# Run `make` or `make help` for commands and quick-start workflows.

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
DATA_DIR     = $(PROJECT_DIR)/data
PYTHON       = export PYTHONPATH="$(PROJECT_DIR)/src:$$PYTHONPATH" && python3
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
	@echo "  Navigation Controller — RRT · A* · TSP"
	@echo "  ────────────────────────────────────────────────────────────────────"
	@echo ""
	@echo "  QUICK START"
	@echo "    make stop && make ros          One terminal: ped + planner + viz"
	@echo "    make ped / planner / viz       Three terminals (simulation)"
	@echo "    make fake-mocap + ped-deploy   Deployment lab (6 m × 6 m, no hardware)"
	@echo "    make tsp-save && make mc       Offline Monte Carlo (no ROS)"
	@echo ""
	@echo "  LIVE STACK"
	@printf "    %-22s %s\n" "make ros" "Full stack in one terminal (~90 s TSP wait)"
	@printf "    %-22s %s\n" "make ped" "Pedestrian sim  [N=$(N_PEDS) SPEED=$(SPEED) HZ=$(HZ)]"
	@printf "    %-22s %s\n" "make ped-deploy" "Deployment viewport  [SPEED=$(SPEED_DEPLOY)]"
	@printf "    %-22s %s\n" "make planner" "RRT + A* planner (needs /pedestrian_state)"
	@printf "    %-22s %s\n" "make viz" "Matplotlib visualizer"
	@printf "    %-22s %s\n" "make stop" "Kill all nodes"
	@echo ""
	@echo "  MOCAP"
	@printf "    %-22s %s\n" "make mocap" "OptiTrack via VRPN  ($(MOCAP_IP):$(MOCAP_PORT))"
	@printf "    %-22s %s\n" "make fake-mocap" "Static poses from config/fake_mocap_poses.json"
	@printf "    %-22s %s\n" "make mocap-echo" "Echo topic  (OBS=$(OBS))"
	@printf "    %-22s %s\n" "make mocap-list" "List /vrpn_mocap/* topics"
	@echo ""
	@echo "  RVIZ"
	@printf "    %-22s %s\n" "make rviz" "Static obstacles + TSP path"
	@printf "    %-22s %s\n" "make rviz-mocap" "Live MoCap obstacles"
	@echo ""
	@echo "  OFFLINE"
	@printf "    %-22s %s\n" "make tsp-save" "Compute TSP → data/tsp_*.npy"
	@printf "    %-22s %s\n" "make tsp" "Plot TSP only"
	@printf "    %-22s %s\n" "make mc" "Monte Carlo: RRT vs A*"
	@printf "    %-22s %s\n" "make mc-astar" "Monte Carlo: A* only"
	@printf "    %-22s %s\n" "make mc5" "Monte Carlo: alternate variant"
	@echo ""
	@echo "  OTHER"
	@printf "    %-22s %s\n" "make copy-deps" "Import reference scripts → archive/import_staging/"
	@echo ""
	@echo "  CONTROLS"
	@echo "    Pedestrian sim:  h add  j remove  u/t speed  p status  q quit"
	@echo "    Planner:         q + Enter quit"
	@echo ""
	@echo "  Override:  make ped N_PEDS=12 SPEED=1.5 HZ=25"
	@echo "  Config:    config/mission_waypoints.json"
	@echo ""

# ═══════════════════════════════════════════════════════════════════════════════
# MOCAP — OptiTrack → ROS2
# ═══════════════════════════════════════════════════════════════════════════════

mocap:
	@echo "MoCap  →  VRPN client  ($(MOCAP_IP):$(MOCAP_PORT))"
	@echo "Topics: /vrpn_mocap/obstacle{1,2,3}/pose"
	@bash -c "$(MOCAP_ENV) && ros2 launch vrpn_mocap client.launch.yaml server:=$(MOCAP_IP) port:=$(MOCAP_PORT)"

mocap-echo:
	@echo "Echoing /vrpn_mocap/$(OBS)/pose  (Ctrl+C to stop)"
	@bash -c "$(MOCAP_ENV) && ros2 topic echo /vrpn_mocap/$(OBS)/pose"

mocap-list:
	@echo "Active MoCap topics:"
	@bash -c "$(MOCAP_ENV) && ros2 topic list | grep vrpn_mocap"

fake-mocap:
	@echo "Fake MoCap  →  config/fake_mocap_poses.json  (reloads every 5 s)"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' $(PYTHON) -m nav_stack.ros.fake_mocap_node"

# ═══════════════════════════════════════════════════════════════════════════════
# OFFLINE — TSP & Monte Carlo
# ═══════════════════════════════════════════════════════════════════════════════

tsp:
	@echo "TSP  →  plot only (no save)"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && $(PYTHON) -m nav_stack.planning.TSP_main"

tsp-save: compute-tsp

compute-tsp:
	@echo "TSP  →  computing global path..."
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && $(PYTHON) -m nav_stack.offline.computeTSP"
	@test -f $(DATA_DIR)/tsp_path.npy && test -f $(DATA_DIR)/tsp_polyline.npy \
		|| (echo "ERROR: data/tsp_path.npy / data/tsp_polyline.npy were not created." && exit 1)
	@echo "Saved  data/tsp_path.npy  data/tsp_polyline.npy"

mc:
	@test -f $(DATA_DIR)/tsp_path.npy \
		|| (echo "ERROR: data/tsp_path.npy not found. Run 'make tsp-save' first." && exit 1)
	@echo "Monte Carlo  →  RRT vs A*"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && $(PYTHON) -m nav_stack.offline.main_sim_TSP_PF_RRT_astar"

mc-astar:
	@test -f $(DATA_DIR)/tsp_path.npy \
		|| (echo "ERROR: data/tsp_path.npy not found. Run 'make tsp-save' first." && exit 1)
	@echo "Monte Carlo  →  A* only"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && $(PYTHON) -m nav_stack.offline.main_MCsim_TSP_PF_Astar"

mc5:
	@test -f $(DATA_DIR)/tsp_path.npy \
		|| (echo "ERROR: data/tsp_path.npy not found. Run 'make tsp-save' first." && exit 1)
	@echo "Monte Carlo  →  alternate variant (main5)"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && $(PYTHON) -m nav_stack.offline.main5_MCsim"

sim: mc

# ═══════════════════════════════════════════════════════════════════════════════
# ROS2 — live stack
# ═══════════════════════════════════════════════════════════════════════════════

ros:
	@echo "ROS stack  →  pedestrian + planner + visualizer"
	@echo "Note: ~90 s TSP wait before visualizer opens"
	@bash $(PROJECT_DIR)/scripts/run_ros2_planner.sh

ped:
	@echo "Pedestrian sim  →  N=$(N_PEDS)  speed=$(SPEED) m/s  hz=$(HZ)"
	@echo "Controls:  h add  j remove  u/t speed  p status  q quit"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' $(PYTHON) -m nav_stack.ros.pedestrian_sim_node --n_pedestrians $(N_PEDS) --speed $(SPEED) --hz $(HZ)"


ped-deploy:
	@echo "Pedestrian sim  →  deployment 6 m × 6 m"
	@echo "  N=$(N_PEDS)  speed=$(SPEED_DEPLOY) m/s  hz=$(HZ)"
	@echo "Controls:  h add  j remove  u/t speed  p status  q quit"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' $(PYTHON) -m nav_stack.ros.pedestrian_sim_node --n_pedestrians $(N_PEDS) --speed $(SPEED_DEPLOY) --hz $(HZ) --deploy"

planner:
	@echo "Planner  →  RRT + A*  (needs /pedestrian_state)"
	@echo "Note: TSP may take ~90 s on first start"
	@echo "Controls:  q + Enter quit"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 $(PYTHON) -u -m nav_stack.ros.planner_node"

viz:
	@echo "Visualizer  →  matplotlib live view"
	@echo "Requires: pedestrian sim + planner running"
	@bash -c "$(ROS_ENV) && cd $(PROJECT_DIR) && RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}' $(PYTHON) -m nav_stack.ros.visualizer_node"

rviz:
	@echo "RViz  →  static obstacles + TSP path"
	@bash $(OBSTACLE_VIZ)/run.sh

rviz-mocap:
	@echo "RViz  →  live MoCap obstacles"
	@echo "Requires: make mocap or make fake-mocap in another terminal"
	@bash $(OBSTACLE_VIZ)/run_mocap.sh

# ═══════════════════════════════════════════════════════════════════════════════
# STOP & UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

stop:
	@echo "Stopping all nodes..."
	@pkill -f nav_stack.ros.pedestrian_sim_node  2>/dev/null || true
	@pkill -f nav_stack.ros.planner_node         2>/dev/null || true
	@pkill -f nav_stack.ros.visualizer_node      2>/dev/null || true
	@pkill -f mocap_obstacle_publisher.py 2>/dev/null || true
	@pkill -f obstacle_publisher.py   2>/dev/null || true
	@pkill -f tsp_path_publisher.py   2>/dev/null || true
	@pkill -f run_ros2_planner.sh     2>/dev/null || true
	@pkill -f rviz2                   2>/dev/null || true
	@pkill -f vrpn_mocap              2>/dev/null || true
	@pkill -f nav_stack.ros.fake_mocap_node      2>/dev/null || true
	@echo "Done."

copy-deps:
	@test -d $(HOME)/Downloads/path_planning-main \
		|| (echo "ERROR: ~/Downloads/path_planning-main not found." && exit 1)
	@echo "Import  →  archive/import_staging/  (merge into src/nav_stack/ manually)"
	@mkdir -p $(PROJECT_DIR)/archive/import_staging
	@cp $(HOME)/Downloads/path_planning-main/tsp_pf_rrt+astar_combined/*.py $(PROJECT_DIR)/archive/import_staging/
	@ls $(PROJECT_DIR)/archive/import_staging/*.py
