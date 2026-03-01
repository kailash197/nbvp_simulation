#!/usr/bin/bash
set -e
# Source ROS and the workspace on every container start
[ -f /opt/ros/${ROS_DISTRO}/setup.bash ] && source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /root/catkin_ws/devel/setup.bash ] && source /root/catkin_ws/devel/setup.bash

exec "$@"
