#!/usr/bin/bash
set -e
# Source ROS and the workspace on every container start
[ -f /opt/ros/${ROS_DISTRO}/setup.bash ] && source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /root/${WS}/devel/setup.bash ] && source /root/${WS}/devel/setup.bash

if [ -f /root/${WS}/session_tmp.yml ]; then
    cp /root/${WS}/session_tmp.yml /root/${WS}/src/nbvp_exploration/startup/kopterworx_one_flying/session.yml
fi

if [ -f ~/${WS}/devel/setup.bash ]; then
    echo "alias ws='source ~/${WS}/devel/setup.bash'" >> ~/.bashrc
    source ~/${WS}/devel/setup.bash
fi

source ~/.bashrc

exec "$@"
