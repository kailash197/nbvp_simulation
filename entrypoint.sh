#!/usr/bin/env bash
set -e
# Source ROS and the workspace on every container start
source /opt/ros/melodic/setup.bash
if [ -f /root/catkin_ws/devel/setup.bash ]; then
  source /root/catkin_ws/devel/setup.bash
fi
exec "$@"
