#!/bin/bash
set -e
source /opt/tros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

MAP=${1:-/root/ros2_ws/maps/originbot_competition_map.yaml}
PARAMS=${2:-/root/ros2_ws/src/originbot_nav2_competition/config/originbot_nav2_competition.yaml}

ros2 launch originbot_nav2_competition nav2_competition.launch.py map:=$MAP params_file:=$PARAMS
