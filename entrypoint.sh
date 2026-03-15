#!/usr/bin/bash
set -e
# Source ROS and the workspace on every container start
[ -f /opt/ros/${ROS_DISTRO}/setup.bash ] && source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /root/${WS}/devel/setup.bash ] && source /root/${WS}/devel/setup.bash

ws="source /opt/ros/${ROS_DISTRO}/setup.bash"
[ -f /root/uav_ws/install/setup.bash ] && ws="${ws} && source /root/uav_ws/install/setup.bash"
[ -f ~/${WS}/devel/setup.bash ] && ws="${ws} && source ~/${WS}/devel/setup.bash --extend"
echo "alias ws='${ws}; source ~/.bashrc'" >> ~/.bashrc

source ~/.bashrc

if [ -f /root/custom_script.bash ]; then
    cd /root/ && ./custom_script.bash
fi

cd /root/${WS}/src/nbvp_exploration/startup/kopterworx_one_flying

exec "$@"
