#!/bin/bash
export PYTHONPATH=""
export LD_LIBRARY_PATH=""
source /opt/ros/jazzy/setup.bash

export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/home/egeozgul/Desktop/luci_ws/install/luci_messages/lib:/usr/local/cuda-12.8/lib64
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
export PYTHONPATH="$PROJECT_DIR/src:${PYTHONPATH:-}"

echo "Starting MoCap obstacle publisher (needs make mocap in another terminal)..."
python3 "$SCRIPT_DIR/mocap_obstacle_publisher.py" &
MOCAP_PID=$!

sleep 2

echo "Starting RViz2 (MoCap view)..."
ros2 run rviz2 rviz2 -d "$SCRIPT_DIR/obstacles_mocap.rviz" &
RVIZ_PID=$!

trap "kill $MOCAP_PID $RVIZ_PID 2>/dev/null" EXIT INT TERM
wait $RVIZ_PID
