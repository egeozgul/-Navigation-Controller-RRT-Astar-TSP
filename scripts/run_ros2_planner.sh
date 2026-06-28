#!/bin/bash
# run_ros2_planner.sh
# Launches: pedestrian_sim_node (foreground, keyboard) + planner + visualizer

source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/home/egeozgul/Desktop/luci_ws/install/luci_messages/lib:/usr/local/cuda-12.8/lib64
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$PROJECT_DIR/src:${PYTHONPATH:-}"
PYTHON="python3 -m"

echo "Starting planner node (TSP computing in background)..."
$PYTHON nav_stack.ros.planner_node &
PLAN_PID=$!

echo "Waiting 90s for TSP to complete..."
sleep 90

echo "Starting visualizer..."
$PYTHON nav_stack.ros.visualizer_node &
VIZ_PID=$!

trap "kill $PLAN_PID $VIZ_PID 2>/dev/null" EXIT INT TERM

echo "Starting pedestrian simulator (foreground — keyboard: + - ] [ p f q)..."
$PYTHON nav_stack.ros.pedestrian_sim_node

kill $PLAN_PID $VIZ_PID 2>/dev/null
