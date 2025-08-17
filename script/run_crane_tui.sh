#!/bin/bash

# Script: run_crane_tui.sh
# Purpose: Run crane_tui.py in Docker with proper volume and environment settings

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Set ROS_DOMAIN_ID environment variable
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Run the container and mount the local script directory (absolute path)
docker run -it --rm --network host \
  -v "$SCRIPT_DIR:/workspaces/script" \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  registry.screamtrumpet.csie.ncku.edu.tw/pros_images/pros_base_image:0.0.5a \
  /bin/bash -c "python3 /workspaces/script/crane_tui.py"

if [ $? -ne 0 ]; then
  echo "[ERROR] Docker container or crane_tui.py failed to run."
  exit 1
fi
