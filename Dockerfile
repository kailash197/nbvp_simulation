## REQUIREMENTS:
# - Ubuntu 18.04
# - ROS Melodic
# - Gazebo simulation of Ardupilot/PX4 UAV platform
# - Path planning and navigation
# - Shadowcasting-based NBV exploration planner

FROM ubuntu:18.04

# Prevent interactive tz dialogs
ENV DEBIAN_FRONTEND=noninteractive

# Basic utilities
RUN apt-get update && apt-get install -y \
    lsb-release \
    curl \
    gnupg2 \
    build-essential

# Add ROS Melodic keys and repo
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1.list'

# Install ROS Melodic base
RUN apt-get update && apt-get install -y ros-melodic-desktop-full

# Source ROS on container startup
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# Create catkin workspace
RUN mkdir -p /root/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && catkin_make"

# Auto-source the catkin workspace too
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Install core ROS Melodic dependencies required for nbvp_exploration
# Test: rosmsg show octomap_msgs/Octomap
RUN apt-get update && apt-get install -y \
    ros-melodic-octomap \
    ros-melodic-octomap-msgs \
    ros-melodic-octomap-ros \
    ros-melodic-tf \
    ros-melodic-tf-conversions \
    ros-melodic-tf2-ros \
    ros-melodic-eigen-conversions \
    ros-melodic-pcl-ros \
    ros-melodic-pcl-conversions \
    ros-melodic-nav-msgs \
    ros-melodic-mav-msgs \
    ros-melodic-geometry-msgs \
    ros-melodic-visualization-msgs \
    ros-melodic-sensor-msgs \
    ros-melodic-message-generation \
    && rm -rf /var/lib/apt/lists/*

# System dependencies needed by catkinized submodules used in nbvp_exploration
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev \
    libgflags-dev \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    git \
    wget \
    tree \
    && rm -rf /var/lib/apt/lists/*

# #############################
# #      NBVP EXPLORATION
# #############################

# Clone catkinized dependencies from nbvp_exploration repository
WORKDIR /root/catkin_ws/src
RUN mkdir -p /root/catkin_ws/src/nbvp_exploration
COPY ./nbvp_exploration /root/catkin_ws/src/nbvp_exploration

SHELL ["/bin/bash", "-lc"]
RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg catkin_simple --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg eigen_catkin --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg gflags_catkin --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg glog_catkin --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg eigen_checks --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg kdtree --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg minkindr --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg minkindr_conversions --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg volumetric_mapping --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg nbvplanner --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /root/catkin_ws/devel/include/eigen3 && \
    catkin_make --pkg interface_nbvp_rotors --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

RUN apt-get update && apt-get install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    ros-melodic-octomap-server \
    ros-melodic-octomap-rviz-plugins \
    ros-melodic-octomap-* \
    ros-melodic-moveit \
    ros-melodic-moveit-visual-tools \
    ros-melodic-dynamixel-workbench-msgs \
    ros-melodic-ompl \
    ros-melodic-rviz \
    libompl-dev \
    mesa-utils \
    x11-apps \
    tmux \
    python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# #############################
# #  LARICS MOTION PLANNING
# #############################
COPY ./aerial_manipulators/aerial_manipulators_description /root/catkin_ws/src/aerial_manipulators/aerial_manipulators_description
RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg aerial_manipulators_description --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash
COPY ./aerial_manipulators/aerial_manipulators_moveit /root/catkin_ws/src/aerial_manipulators/aerial_manipulators_moveit
RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg asap_manipulator --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg wp_manipulator --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg wp_manipulator_3r --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg wp_manipulator_3rx --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash



COPY ./larics_gazebo_worlds /root/catkin_ws/src/larics_gazebo_worlds
RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg larics_gazebo_worlds --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

COPY ./topp_ros /root/catkin_ws/src/topp_ros
RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg topp_ros --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

COPY ./larics_motion_planning /root/catkin_ws/src/larics_motion_planning
RUN cd /root/catkin_ws/src/larics_motion_planning && \
    git checkout exploration && \
    # The following dependencies have been removed in exploration branch.
    # 239171d Remove dependency on aerial manipulators and impedance control repositories
    source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg larics_motion_planning --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

# #############################
# #     UAV ROS SIMULATION
# #############################
ENV ROS_DISTRO melodic
ENV distro 18.04

# 1. install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends tzdata  dialog apt-utils && \
    apt-get -y install gnupg2 libterm-readline-gnu-perl lsb-release && \
    apt-get -y upgrade --fix-missing && \
    apt-get -y install python-pip python3-pip python-setuptools python3-setuptools && \
    rm -rf /var/lib/apt/lists/*

RUN sleep 90 && \
    { sudo systemctl stop google-instance-setup.service && \
        echo "gce service stopped" || \
        echo "gce service not stopped"; } && \
    { sudo timeout 120s apt-get -y install gce-compute-image-packages || \
        echo "\e[1;31mInstallation of gce-compute-image-packages failed\e[0m"; }

# Dont install dependencies/ros.sh, already installed in ros-melodic-desktop-full
# bash $MY_PATH/dependencies/ros.sh

# install gitman and use it to install dependencies of uav_ros_simulation
# bash $MY_PATH/dependencies/gitman.sh
RUN pip3 install gitman && \
    sudo -H pip3 install gitman

COPY ./uav_ros_simulation /root/catkin_ws/src/uav_ros_simulation
COPY ./larics_uav_ros_simulation /root/catkin_ws/src/larics_uav_ros_simulation
WORKDIR /root/catkin_ws/src/uav_ros_simulation/installation
RUN gitman install --force

# WORKDIR /root/catkin_ws/src/uav_ros_simulation/
# RUN rm -f firmware/ardupilot && \
#     git clone --recursive https://github.com/larics/ardupilot.git ./firmware/ardupilot && \
#     ln -sf ../.gitman/ardupilot firmware/ardupilot && \
#     cd /root/catkin_ws/src/uav_ros_simulation/firmware/ardupilot && \
#     git checkout master && \
#     git submodule update --init --recursive


# RUN /root/catkin_ws/src/uav_ros_simulation/installation/simple_install.sh
# missing repo
# https://github.com/lmark1/uav_ros_stack
# maybe moved to following:
# https://github.com/mkrizmancic/uav_ros_stack

# RUN mkdir -p /root/catkin_ws/src/uav_ros_simulation
# COPY ./uav_ros_simulation /root/catkin_ws/src/uav_ros_simulation


# RUN pip3 install gitman && \
#     sudo -H pip3 install gitman

# # WORKDIR /root/catkin_ws/src/uav_ros_simulation/installation
# # RUN gitman install --force

# #############################
# #     VELODYNE SIMULATION
# #############################


WORKDIR /root/catkin_ws
COPY ./entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh
ENTRYPOINT [ "/root/entrypoint.sh" ]
CMD ["/bin/bash"]

# https://github.com/RobertMilijas/uav_ros_simulation.git