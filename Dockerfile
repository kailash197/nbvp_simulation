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
# Step 4 done

# Step 5
# Clone catkinized dependencies from nbvp_exploration repository
RUN cd /root/catkin_ws/src && \
    git clone https://github.com/larics/nbvp_exploration.git nbvp_repo && \
    cp -r nbvp_repo/catkin_simple . && \
    cp -r nbvp_repo/glog_catkin . && \
    cp -r nbvp_repo/gflags_catkin . && \
    cp -r nbvp_repo/eigen_catkin . && \
    cp -r nbvp_repo/eigen_checks . && \
    cp -r nbvp_repo/kdtree . && \
    cp -r nbvp_repo/minkindr . && \
    cp -r nbvp_repo/minkindr_ros . && \
    cp -r nbvp_repo/volumetric_mapping . && \
    cp -r nbvp_repo/nbvplanner . && \
    cp -r nbvp_repo/interface_nbvp_rotors . && \
    rm -rf nbvp_repo

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
    catkin_make --pkg volumetric_mapping --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg nbvplanner --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    rm -rf /root/catkin_ws/devel/include/eigen3 && \
    catkin_make --pkg interface_nbvp_rotors --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN source /root/catkin_ws/devel/setup.bash

RUN apt-get update && apt-get install -y \
    ros-melodic-rviz \
    mesa-utils \
    x11-apps \
    tmux \
    && rm -rf /var/lib/apt/lists/*

RUN cd /root/catkin_ws/src && \
    git clone https://github.com/larics/nbvp_exploration.git nbvp_startup_tmp && \
    mkdir -p nbvp_exploration_startup && \
    cp -r nbvp_startup_tmp/startup/kopterworx_one_flying nbvp_exploration_startup/ && \
    rm -rf nbvp_startup_tmp

RUN apt-get update && apt-get install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*


# RUN cd /root/catkin_ws/src && \
#     git clone https://github.com/larics/topp_ros.git && \
#     source /opt/ros/melodic/setup.bash && \
#     cd /root/catkin_ws && \
#     catkin_make --pkg topp_ros --cmake-args -DCMAKE_BUILD_TYPE=Release && \
#     source /root/catkin_ws/devel/setup.bash

RUN apt-get update && apt-get install -y \
    ros-melodic-octomap-server ros-melodic-octomap-rviz-plugins \
    ros-melodic-moveit \
    ros-melodic-moveit-visual-tools \
    ros-melodic-dynamixel-workbench-msgs \
    ros-melodic-ompl \
    libompl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd /root/catkin_ws/src && \
    git clone -b devel https://github.com/larics/aerial_manipulators.git && \
    mv ./aerial_manipulators/aerial_manipulators_description/ . && \
    rm -rf ./aerial_manipulators && \
    source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg aerial_manipulators_description --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

RUN cd /root/catkin_ws/src && \
    git clone -b devel https://github.com/larics/aerial_manipulators.git && \
    mv ./aerial_manipulators/aerial_manipulators_moveit/ . && \
    rm -rf ./aerial_manipulators && \
    source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg asap_manipulator --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg wp_manipulator --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg wp_manipulator_3r --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin_make --pkg wp_manipulator_3rx --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

RUN cd /root/catkin_ws/src && \
    git clone https://github.com/larics/larics_gazebo_worlds.git && \
    source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg larics_gazebo_worlds --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

# Stub package for impedaance control
RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws/src && \
    mkdir -p impedance_control
COPY ./impedance_control/package.xml /root/catkin_ws/src/impedance_control/package.xml
COPY ./impedance_control/CMakeLists.txt /root/catkin_ws/src/impedance_control/CMakeLists.txt
RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg impedance_control --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

# Stub package for aerial_manipulators_control
RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws/src && \
    mkdir -p aerial_manipulators_control/include/aerial_manipulators_control
COPY ./aerial_manipulators_control/package.xml /root/catkin_ws/src/aerial_manipulators_control/package.xml
COPY ./aerial_manipulators_control/CMakeLists.txt /root/catkin_ws/src/aerial_manipulators_control/CMakeLists.txt
COPY ./aerial_manipulators_control/include/aerial_manipulators_control/stub.h /root/catkin_ws/src/aerial_manipulators_control/include/aerial_manipulators_control/stub.h
RUN source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg aerial_manipulators_control --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

# RUN cd /root/catkin_ws/src && \
#     git clone -b devel https://github.com/larics/aerial_manipulators.git && \
#     mv ./aerial_manipulators/aerial_manipulators_control/ ./ && \
#     rm -rf ./aerial_manipulators && \
#     source /opt/ros/melodic/setup.bash && \
#     cd /root/catkin_ws && \
#     catkin_make --pkg aerial_manipulators_control --cmake-args -DCMAKE_BUILD_TYPE=Release && \
#     source /root/catkin_ws/devel/setup.bash

RUN cd /root/catkin_ws/src && \
    git clone https://github.com/larics/topp_ros.git && \
    source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg topp_ros --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /root/catkin_ws/devel/setup.bash

# RUN cd /root/catkin_ws/src && \
#     git clone https://github.com/larics/larics_motion_planning.git && \
#     source /opt/ros/melodic/setup.bash && \
#     cd /root/catkin_ws && \
#     catkin_make --pkg topp_ros --cmake-args -DCMAKE_BUILD_TYPE=Release -Dompl_DIR=/usr/share/ompl/ompl-config.cmake && \
#     source /root/catkin_ws/devel/setup.bash

WORKDIR /root/catkin_ws
COPY ./entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh
ENTRYPOINT [ "/root/entrypoint.sh" ]
CMD ["/bin/bash"]
