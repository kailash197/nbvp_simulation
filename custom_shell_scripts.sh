# ROS shell scripts
# credit to https://github.com/ctu-mrs/mrs_uav_system

waitForRos() {
  until rostopic list > /dev/null 2>&1; do
    echo "waiting for ros"
    sleep 1;
  done
}

waitForSimulation() {
  until rostopic echo /gazebo/model_states -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for simulation"
    sleep 1;
  done
  sleep 1;
}

waitForOdometry() {
  until rostopic echo /$UAV_NAMESPACE/mavros/local_position/odom -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for odometry"
    sleep 1;
  done
}

waitForCarrot() {
  until rostopic echo /$UAV_NAMESPACE/carrot/status -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for carrot"
    sleep 1;
  done
}

wait_roscore() {
  while [[ ! -f "/tmp/roscoreX" ]]; do
    sleep 1
  done
}
