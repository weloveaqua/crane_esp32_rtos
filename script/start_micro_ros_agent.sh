
#!/bin/bash

# Script: start_micro_ros_agent.sh
# Purpose: Start micro-ROS agent in Docker with configurable port and ROS_DISTRO

# Default values
PORT=${1:-8888}
ROS_DISTRO=${ROS_DISTRO:-humble}

echo "[INFO] Starting micro-ROS agent on UDP port $PORT with ROS_DISTRO=$ROS_DISTRO"

# Run agent container
docker run -it --rm \
  -p $PORT:$PORT/udp \
  microros/micro-ros-agent:$ROS_DISTRO \
  udp4 --port $PORT -v4

if [ $? -ne 0 ]; then
  echo "[ERROR] micro-ROS agent failed to start."
  exit 1
fi

# Usage:
#   bash start_micro_ros_agent.sh [PORT]
#   export ROS_DISTRO=foxy
#   Default port is 8888, default ROS_DISTRO is foxy
