# **<font color='green'>Arm & Takeoff</font>** 
```bash
roslaunch uav_ros_control pid_carrot.launch manual_takeoff:=false
rosrun ardupilot_gazebo automatic_takeoff.sh 0.7
```

## Node 1. 
Node [/red/carrot_reference_node]
Publications: 
 * /red/carrot/cmd_vel [geometry_msgs/TwistStamped]
 * /red/carrot/control_error [uav_ros_msgs/ControlError]
 * /red/carrot/pose [geometry_msgs/PoseStamped]
 * /red/carrot/status [std_msgs/String]
 * /red/carrot/trajectory [trajectory_msgs/MultiDOFJointTrajectoryPoint]
 * /red/carrot/yaw [std_msgs/Float64]
 * /red/uav/yaw [std_msgs/Float64]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /red/joy [unknown type]
 * /red/mavros/global_position/local [nav_msgs/Odometry]
 * /red/mavros/state [mavros_msgs/State]
 * /red/position_hold/trajectory [trajectory_msgs/MultiDOFJointTrajectoryPoint]

Services: 
 * /red/carrot_reference_node/get_loggers
 * /red/carrot_reference_node/set_logger_level
 * /red/land
 * /red/position_hold
 * /red/takeoff

 Connections:
 * topic: /red/carrot/trajectory
    * to: /red/pid_cascade_node
    * direction: outbound (57503 - 127.0.0.1:35448) [10]
    * transport: TCPROS
 * topic: /red/carrot/trajectory
    * to: /red/toppra_uav_ros_tracker
    * direction: outbound (57503 - 127.0.0.1:41154) [18]
    * transport: TCPROS
 * topic: /red/carrot/pose
    * to: /red/toppra_uav_ros_tracker
    * direction: outbound (57503 - 127.0.0.1:41170) [19]
    * transport: TCPROS
 * topic: /red/carrot/status
    * to: /red/pid_cascade_node
    * direction: outbound (57503 - 127.0.0.1:35450) [12]
    * transport: TCPROS
 * topic: /red/carrot/status
    * to: /red/toppra_uav_ros_tracker
    * direction: outbound (57503 - 127.0.0.1:41148) [14]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound (52802 - YOGA:54595) [11]
    * transport: TCPROS
 * topic: /red/mavros/state
    * to: /red/mavros (http://YOGA:40395/)
    * direction: inbound (51184 - YOGA:44233) [16]
    * transport: TCPROS
 * topic: /red/mavros/global_position/local
    * to: /red/mavros (http://YOGA:40395/)
    * direction: inbound (51192 - YOGA:44233) [17]
    * transport: TCPROS
 * topic: /red/position_hold/trajectory
    * to: /red/toppra_uav_ros_tracker (http://YOGA:45373/)
    * direction: inbound (33688 - YOGA:42127) [21]
    * transport: TCPROS


 ## Node 2.
 Node [/red/pid_cascade_node]
Publications: 
 * /red/carrot/velocity [geometry_msgs/Vector3]
 * /red/cascade_config/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /red/cascade_config/parameter_updates [dynamic_reconfigure/Config]
 * /red/mavros/setpoint_raw/attitude [mavros_msgs/AttitudeTarget]
 * /red/uav/euler_setpoint [geometry_msgs/Vector3]
 * /red/uav/setpoint/attitude_global [mavros_msgs/AttitudeTarget]
 * /red/uav/velocity [geometry_msgs/Vector3]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /red/carrot/status [std_msgs/String]
 * /red/carrot/trajectory [trajectory_msgs/MultiDOFJointTrajectoryPoint]
 * /red/mavros/global_position/local [nav_msgs/Odometry]

Services: 
 * /red/cascade_config/set_parameters
 * /red/pid_cascade_node/get_loggers
 * /red/pid_cascade_node/set_logger_level
 * /red/reset_integrator

Connections:
 * topic: /red/mavros/setpoint_raw/attitude
    * to: /red/mavros
    * direction: outbound (54165 - 127.0.0.1:40578) [11]
    * transport: TCPROS
 * topic: /red/mavros/global_position/local
    * to: /red/mavros (http://YOGA:40395/)
    * direction: inbound (51204 - YOGA:44233) [17]
    * transport: TCPROS
 * topic: /red/carrot/trajectory
    * to: /red/carrot_reference_node (http://YOGA:44081/)
    * direction: inbound (35448 - YOGA:57503) [18]
    * transport: TCPROS
 * topic: /red/carrot/status
    * to: /red/carrot_reference_node (http://YOGA:44081/)
    * direction: inbound (35450 - YOGA:57503) [13]
    * transport: TCPROS


# **<font color='green'>Exploration</font>**
- This launch file sets up a **complete autonomous exploration system** using NBVP (Next Best View Planning).
- Creates a full autonomy pipeline for a UAV to explore unknown environments by continuously selecting and flying to the best next viewing positions.
```bash
roslaunch interface_nbvp_rotors kopterworx_exploration.launch
```

## Node 1. **TF Node** (`interface_nbvp_rotors/tf_node`)
- Publishes the transform between `mavros/world` (global frame) and `red/base_link` (UAV body frame)
- Essential for coordinate frame alignment
- Uses odometry data from `mavros/global_position/local`

Node [/red/tf_node]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /red/mavros/global_position/local [nav_msgs/Odometry]

Services: 
 * /red/tf_node/get_loggers
 * /red/tf_node/set_logger_level


contacting node http://YOGA:42593/ ...
Pid: 6227
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (47181 - 127.0.0.1:56220) [11]
    * transport: TCPROS
 * topic: /tf
    * to: /gazebo
    * direction: outbound (47181 - 127.0.0.1:56228) [14]
    * transport: TCPROS
 * topic: /tf
    * to: /red/mavros
    * direction: outbound (47181 - 127.0.0.1:56242) [15]
    * transport: TCPROS
 * topic: /tf
    * to: /red/nbvPlanner
    * direction: outbound (47181 - 127.0.0.1:56250) [16]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound (33162 - YOGA:54595) [13]
    * transport: TCPROS
 * topic: /red/mavros/global_position/local
    * to: /red/mavros (http://YOGA:40395/)
    * direction: inbound (37538 - YOGA:44233) [10]
    * transport: TCPROS


## Node 2. **NBV Planner** (`nbvplanner/nbvPlanner`)
The core intelligence of the system:
- **Purpose**: Determines the "Next Best View" - the most informative next position to explore
- **Input**: 
  - `velodyne_points` - LiDAR point cloud data
  - Odometry for position
- **Parameters**:
  - `resolution`: 0.2m grid resolution for mapping
  - `sensor_max_range`: 20.0m LiDAR range
  - `probability_hit`: 0.9 confidence for occupancy mapping
  - `visualize_max_z`: 2.5m max height for visualization
- **Output**: Publishes candidate viewpoints for exploration
--------------------------------------------------------------------------------
Node [/red/nbvPlanner]
Publications: 
 * /red/comp_times [std_msgs/Float64MultiArray]
 * /red/inspectionPath [visualization_msgs/Marker]
 * /red/nbvPlanner/nearest_obstacle [sensor_msgs/PointCloud2]
 * /red/nbvPlanner/octomap_binary [octomap_msgs/Octomap]
 * /red/nbvPlanner/octomap_free [visualization_msgs/MarkerArray]
 * /red/nbvPlanner/octomap_full [octomap_msgs/Octomap]
 * /red/nbvPlanner/octomap_occupied [visualization_msgs/MarkerArray]
 * /red/nbvPlanner/octomap_pcl [sensor_msgs/PointCloud2]
 * /red/octomap_volume [std_msgs/Float64MultiArray]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /red/cam0/camera_info [unknown type]
 * /red/cam1/camera_info [unknown type]
 * /red/disparity [unknown type]
 * /red/input_octomap [unknown type]
 * /red/mavros/global_position/local [nav_msgs/Odometry]
 * /red/peer_pose_1 [unknown type]
 * /red/peer_pose_2 [unknown type]
 * /red/peer_pose_3 [unknown type]
 * /red/pointcloud [unknown type]
 * /red/pointcloud_throttled_down [unknown type]
 * /red/pointcloud_throttled_up [unknown type]
 * /red/smece [unknown type]
 * /red/velodyne_points [sensor_msgs/PointCloud2]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Services: 
 * /red/nbvPlanner/get_loggers
 * /red/nbvPlanner/get_map
 * /red/nbvPlanner/load_map
 * /red/nbvPlanner/publish_all
 * /red/nbvPlanner/reset_map
 * /red/nbvPlanner/save_map
 * /red/nbvPlanner/save_point_cloud
 * /red/nbvPlanner/set_box_occupancy
 * /red/nbvPlanner/set_display_bounds
 * /red/nbvPlanner/set_logger_level
 * /red/nbvplanner
 * /red/test
 * /red/volume_service


contacting node http://YOGA:36419/ ...
Pid: 6228
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (36937 - 127.0.0.1:48758) [19]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound (33176 - YOGA:54595) [18]
    * transport: TCPROS
 * topic: /tf
    * to: /red/mavros (http://YOGA:40395/)
    * direction: inbound (37568 - YOGA:44233) [22]
    * transport: TCPROS
 * topic: /tf
    * to: /red/robot_state_publisher (http://YOGA:43439/)
    * direction: inbound (35370 - YOGA:55289) [23]
    * transport: TCPROS
 * topic: /tf
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound (33184 - YOGA:54595) [24]
    * transport: TCPROS
 * topic: /tf
    * to: /red/tf_node (http://YOGA:42593/)
    * direction: inbound (56250 - YOGA:47181) [25]
    * transport: TCPROS
 * topic: /tf_static
    * to: /red/mavros (http://YOGA:40395/)
    * direction: inbound (37556 - YOGA:44233) [20]
    * transport: TCPROS
 * topic: /tf_static
    * to: /red/robot_state_publisher (http://YOGA:43439/)
    * direction: inbound (35366 - YOGA:55289) [21]
    * transport: TCPROS
 * topic: /red/mavros/global_position/local
    * to: /red/mavros (http://YOGA:40395/)
    * direction: inbound (37582 - YOGA:44233) [12]
    * transport: TCPROS
 * topic: /red/velodyne_points
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound (33198 - YOGA:54595) [10]
    * transport: TCPROS



## Node 3. **Exploration Node** (`interface_nbvp_rotors/exploration`)
Manages the exploration mission:
- Receives NBV planner's suggestions
- Generates trajectories to the next best view
- **Output**: Publishes trajectory points to `position_hold/trajectory`[`trajectory_msgs/MultiDOFJointTrajectoryPoint`]

Node [/red/exploration]
Publications: 
 * /red/nbvp/goals [geometry_msgs/PoseArray]
 * /red/position_hold/trajectory [trajectory_msgs/MultiDOFJointTrajectoryPoint]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /red/nbvp/point_reached [std_msgs/Bool]

Services: 
 * /red/exploration/get_loggers
 * /red/exploration/set_logger_level


contacting node http://YOGA:33271/ ...
Pid: 6229
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (35333 - 127.0.0.1:58710) [16]
    * transport: TCPROS
 * topic: /red/position_hold/trajectory
    * to: /red/carrot_reference_node
    * direction: outbound (35333 - 127.0.0.1:58724) [12]
    * transport: TCPROS
 * topic: /red/nbvp/goals
    * to: /red/execute_trajectory
    * direction: outbound (35333 - 127.0.0.1:58720) [10]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound (33166 - YOGA:54595) [15]
    * transport: TCPROS
 * topic: /red/nbvp/point_reached
    * to: /red/execute_trajectory (http://YOGA:37371/)
    * direction: inbound (50802 - YOGA:42471) [11]
    * transport: TCPROS



---
---

# **<font color='green'>Execute Trajectory</font>**
- This launch file runs a **trajectory execution state machine** for UAVs.
- Executes pre-planned trajectories with feedback monitoring and completion verification.
```bash
roslaunch interface_nbvp_rotors execute_trajectory.launch
```
## Node 1. Trajectory Executor Node (`execute_trajectory_state_machine.py`)
- A state machine that manages the actual flight execution:
- Reads TrajectoryPoint, service call to get trajectory based on TrajectoryPoint, and publish trajectory to execute
- Takes a planned trajectory and commands the UAV to fly it
- Monitors execution progress
- Verifies when the trajectory is complete
- Handles state transitions during flight
 
- Subscribes: `carrot/trajectory` [`MultiDOFJointTrajectoryPoint`] [PLAN]
- Service call: `multi_dof_trajectory` [`MultiDofTrajectory'] [PLAN]
- Publish: `joint_trajectory` [`JointTrajectory`] [EXECUTE]

## Role in the Complete System
- Manages the **high-level state** of trajectory execution
- Determines **when the mission is complete** (within 0.7m radius)
- Collects **feedback data** after execution
- Handles **error cases** and state transitions
Node [/red/execute_trajectory]
Publications: 
 * /red/exploration/current_state [std_msgs/String]
 * /red/joint_trajectory [trajectory_msgs/JointTrajectory]
 * /red/nbvp/point_reached [std_msgs/Bool]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /red/carrot/trajectory [trajectory_msgs/MultiDOFJointTrajectoryPoint]
 * /red/executing_trajectory [std_msgs/Int32]
 * /red/mavros/global_position/local [nav_msgs/Odometry]
 * /red/nbvp/goals [unknown type]
 * /red/nbvp/target [unknown type]

Services: 
 * /red/execute_trajectory/get_loggers
 * /red/execute_trajectory/set_logger_level


contacting node http://YOGA:37371/ ...
Pid: 6075
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (42471 - 127.0.0.1:54720) [17]
    * transport: TCPROS
 * topic: /red/joint_trajectory
    * to: /red/joint_trajectory_to_multi_dof_trajectory_point
    * direction: outbound (42471 - 127.0.0.1:54718) [9]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound
    * transport: TCPROS
 * topic: /red/carrot/trajectory
    * to: /red/carrot_reference_node (http://YOGA:44081/)
    * direction: inbound
    * transport: TCPROS
 * topic: /red/mavros/global_position/local
    * to: /red/mavros (http://YOGA:40395/)
    * direction: inbound
    * transport: TCPROS
 * topic: /red/executing_trajectory
    * to: /red/joint_trajectory_to_multi_dof_trajectory_point (http://YOGA:33781/)
    * direction: inbound
    * transport: TCPROS

---
---

# **<font color='green'>Planner</font>**
- This ROS launch file sets up a **complete trajectory planning pipeline** for a UAV.
- Creates a system that plans global paths, generates time-optimal trajectories, and converts them into reference points for the UAV controller.
```bash
roslaunch interface_nbvp_rotors planner.launch
```

## Node 1. **TOPP Trajectory Generator** (`topp_ros/generate_toppra_trajectory.py`)
- Generates time-optimal trajectories using TOPP algorithm

- Service Server: 
  - `generate_toppra_trajectory` [GenerateTrajectory]
  - generates time-optimal trajectories from waypoints

- Publishers
  - `toppra_raw_trajectory` (JointTrajectory) - outputs optimized trajectory
  - `toppra_raw_waypoints` (JointTrajectory) - outputs input waypoints for debugging

Node [/red/generate_toppra_trajectory]
Publications: 
 * /red/toppra_raw_trajectory [trajectory_msgs/JointTrajectory]
 * /red/toppra_raw_waypoints [trajectory_msgs/JointTrajectory]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]

Services: 
 * /red/generate_toppra_trajectory
 * /red/generate_toppra_trajectory/get_loggers
 * /red/generate_toppra_trajectory/set_logger_level


contacting node http://YOGA:38969/ ...
Pid: 5772
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (46857 - 127.0.0.1:47628) [13]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound
    * transport: TCPROS

## Node 2. **Global Planner Node** (`larics_motion_planning/global_planner_node`)
This is the key component that **supplies the path**:
- **Purpose**: Plans global paths through the environment
- **Input**: Listens to `/octomap_binary` (3D occupancy map of the environment)
- **Configuration**: Loads settings from `kopterworx_config.yaml`
- **Function**: Generates collision-free paths from start to goal positions

Node [/red/global_planner]
Publications: 
 * /red/cartesian_path [nav_msgs/Path]
 * /red/joint_trajectory [trajectory_msgs/JointTrajectory]
 * /red/multi_dof_trajectory [trajectory_msgs/MultiDOFJointTrajectory]
 * /red/parabolic_airdrop/info_vector [std_msgs/Float64MultiArray]
 * /red/path_as_joint_trajectory [trajectory_msgs/JointTrajectory]
 * /red/visualization/path [nav_msgs/Path]
 * /red/visualization/state_points [visualization_msgs/Marker]
 * /red/visualization/trajectory [nav_msgs/Path]
 * /red/visualization/waypoints [visualization_msgs/Marker]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /red/octomap_binary [unknown type]

Services: 
 * /red/cartesian_trajectory
 * /red/global_planner/get_loggers
 * /red/global_planner/set_logger_level
 * /red/model_correction_trajectory
 * /red/multi_dof_trajectory
 * /red/multiple_manipulators_model_correction_trajectory
 * /red/multiple_manipulators_object_trajectory
 * /red/parabolic_airdrop_trajectory
 * /red/save_octomap
 * /red/validity_checker
 * /red/visualize_state


contacting node http://YOGA:35681/ ...
Pid: 5773
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (55819 - 127.0.0.1:55202) [10]
    * transport: TCPROS
 * topic: /red/joint_trajectory
    * to: /red/joint_trajectory_to_multi_dof_trajectory_point
    * direction: outbound (55819 - 127.0.0.1:38732) [11]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound (35466 - YOGA:54595) [13]
    * transport: TCPROS


## Node 3. **Trajectory Converter** (`joint_trajectory_to_multi_dof_trajectory_point.py`)
- Converts the planned trajectory into reference points for the controller
- Translates JointTrajectory → MultiDOFJointTrajectoryPoint
- Publishes one point per loop until trajectory complete
- Runs at 100 Hz (high frequency for smooth control)
- Subscribers:
  - `joint_trajectory` [JointTrajectory] - Receives arm-style trajectory to convert
  - `pose` [PoseStamped] - Current UAV position (for airdrop mode)
  - `velocity_relative` [TwistStamped] - Current UAV velocity (for airdrop mode)

- Publishers:
  - `trajectory_point_ref` remapped to  `position_hold/trajectory` [MultiDOFJointTrajectoryPoint] - Outputs converted UAV-style trajectory points
  - `executing_trajectory` [Int32] - Publishes 1 when executing, 0 when idle
  - `magnet/gain` [Float32] - Magnet on/off control for payload release (airdrop mode)
  - `dropoff_delta/position` [Pose] - Distance to airdrop point (airdrop mode)
  - `dropoff_delta/velocity` [Twist] - Velocity difference to airdrop point (airdrop mode)


Node [/red/joint_trajectory_to_multi_dof_trajectory_point]
Publications: 
 * /red/executing_trajectory [std_msgs/Int32]
 * /red/position_hold/trajectory [trajectory_msgs/MultiDOFJointTrajectoryPoint]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /red/joint_trajectory [trajectory_msgs/JointTrajectory]

Services: 
 * /red/joint_trajectory_to_multi_dof_trajectory_point/get_loggers
 * /red/joint_trajectory_to_multi_dof_trajectory_point/set_logger_level


contacting node http://YOGA:33781/ ...
Pid: 5774
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (43117 - 127.0.0.1:49326) [11]
    * transport: TCPROS
 * topic: /red/position_hold/trajectory
    * to: /red/carrot_reference_node
    * direction: outbound (43117 - 127.0.0.1:49330) [8]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound
    * transport: TCPROS
 * topic: /red/joint_trajectory
    * to: /red/global_planner (http://YOGA:35681/)
    * direction: inbound
    * transport: TCPROS


## Data Flow
1. **Global Planner** receives octomap → Plans path
2. **TOPP Generator** creates time-optimal trajectory along path
3. **Converter** transforms trajectory → high-frequency reference points
4. **Output** goes to `position_hold/trajectory` for the tracker to follow



---
---

# **<font color='green'>Trajectory Tracker</font>**
- This ROS launch file sets up a trajectory tracking system for a UAV (Unmanned Aerial Vehicle).  
- It launches two nodes that work together to make a UAV follow a planned trajectory while maintaining smooth motion constraints.
- This system enables precise trajectory tracking for UAVs by generating time-optimal paths and using a carrot-following approach to guide the aircraft along the desired path while respecting dynamic constraints.
```bash
roslaunch uav_ros_tracker topp_tracker.launch tracker_config:=./custom_config/topp_config_custom.yaml
```

## Node 1. **TOPP Trajectory Generator Node** (`topp_ros/generate_toppra_trajectory.py`)
- Generates time-optimal trajectories using TOPP (Time-Optimal Path Parameterization) algorithm
### Main Tasks:
1. **Provide TOPP-RA optimization service** - Listens for trajectory generation requests
2. **Extract waypoints and constraints** - Reads positions, velocity/acceleration limits from request
3. **Run TOPP-RA algorithm** - Computes time-optimal path parameterization
4. **Generate smooth trajectories** - Creates spline interpolation through waypoints
5. **Sample and convert to ROS messages** - Converts algorithm output to JointTrajectory at specified frequency
6. **Add final rest point** - Ensures trajectory ends with zero velocity/acceleration
7. **Publish debug data** - Raw trajectories and waypoints for monitoring
8. **Optional visualization** - Plots acceleration profiles and feasible sets when requested

### Service Server:
- **`generate_toppra_trajectory`** (GenerateTrajectory) - Generates time-optimal trajectories from waypoints with velocity/acceleration constraints

### Publishers:
- **`toppra_raw_trajectory`** (JointTrajectory) - Outputs the optimized trajectory
- **`toppra_raw_waypoints`** (JointTrajectory) - Publishes input waypoints for debugging


Node [/red/topp_trajectory_gen]
Publications: 
 * /red/toppra_raw_trajectory [trajectory_msgs/JointTrajectory]
 * /red/toppra_raw_waypoints [trajectory_msgs/JointTrajectory]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]

Services: 
 * /red/generate_toppra_trajectory
 * /red/topp_trajectory_gen/get_loggers
 * /red/topp_trajectory_gen/set_logger_level


## Node 2. **TOPPRA UAV Tracker Node** (`uav_ros_tracker/topp_tracker.py`)
- Implements "carrot chasing" algorithm where the UAV follows a moving target point along the trajectory

### Main Tasks:
1. **Receive input trajectories** - Accepts multi-DOF trajectories or single poses
2. **Interpolate from current position** - Creates smooth path from carrot to first waypoint
3. **Optimize with TOPP-RA** - Calls TOPP-RA service for time-optimal trajectory generation
4. **Convert joint to multi-DOF** - Translates TOPP-RA output back to multi-DOF format
5. **Publish trajectory points** - Executes trajectory point-by-point at sampling frequency
6. **Manage execution permissions** - Handles enable/disable based on carrot status and user permission
7. **Visualize trajectory** - Publishes path and remaining points for RViz

### Subscribers:
- **`tracker/input_trajectory`** (MultiDOFJointTrajectory) - Input multi-DOF trajectory to track
- **`tracker/input_pose`** (PoseStamped) - Single pose input (converted to 1-point trajectory)
- **`carrot/status`** (String) - Position hold status from carrot controller
- **`carrot/trajectory`** (MultiDOFJointTrajectoryPoint) - Current carrot position (starting point)
- **`carrot/pose`** (PoseStamped) - Current carrot pose (alternative)
- **`odometry_topic`** (Odometry) - UAV odometry (when using odom mode)

### Publishers:
- **`output/point`** (MultiDOFJointTrajectoryPoint) - Output trajectory points for execution
- **`output/pose`** (PoseStamped) - Output pose for visualization
- **`tracker/status`** (String) - Tracker status (OFF/ACCEPT/WAIT/ACTIVE)
- **`tracker/path`** (Path) - Complete planned path for visualization
- **`tracker/remaining_trajectory`** (PoseArray) - Downsampled remaining trajectory points

### Service Servers (Provided):
- **`tracker/enable`** (SetBool) - Enable/disable trajectory publishing
- **`tracker/reset`** (Empty) - Clear current trajectory

### Service Client (Calls):
- **`generate_toppra_trajectory`** (GenerateTrajectory) - Calls TOPP-RA for time optimization

Node [/red/toppra_uav_ros_tracker]
Publications: 
 * /red/output/pose [geometry_msgs/PoseStamped]
 * /red/position_hold/trajectory [trajectory_msgs/MultiDOFJointTrajectoryPoint]
 * /red/tracker/path [nav_msgs/Path]
 * /red/tracker/remaining_trajectory [geometry_msgs/PoseArray]
 * /red/tracker/status [std_msgs/String]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /red/carrot/pose [geometry_msgs/PoseStamped]
 * /red/carrot/status [std_msgs/String]
 * /red/carrot/trajectory [trajectory_msgs/MultiDOFJointTrajectoryPoint]
 * /red/mavros/global_position/local [nav_msgs/Odometry]
 * /red/tracker/input_pose [unknown type]
 * /red/tracker/input_trajectory [unknown type]

Services: 
 * /red/toppra_uav_ros_tracker/get_loggers
 * /red/toppra_uav_ros_tracker/set_logger_level
 * /red/tracker/enable
 * /red/tracker/reset

 Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (42127 - 127.0.0.1:33680) [10]
    * transport: TCPROS
 * topic: /red/position_hold/trajectory
    * to: /red/carrot_reference_node
    * direction: outbound (42127 - 127.0.0.1:33688) [27]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://YOGA:34783/)
    * direction: inbound
    * transport: TCPROS
 * topic: /red/carrot/status
    * to: /red/carrot_reference_node (http://YOGA:44081/)
    * direction: inbound
    * transport: TCPROS
 * topic: /red/carrot/trajectory
    * to: /red/carrot_reference_node (http://YOGA:44081/)
    * direction: inbound
    * transport: TCPROS
 * topic: /red/carrot/pose
    * to: /red/carrot_reference_node (http://YOGA:44081/)
    * direction: inbound
    * transport: TCPROS
 * topic: /red/mavros/global_position/local
    * to: /red/mavros (http://YOGA:40395/)
    * direction: inbound
    * transport: TCPROS

---
---

# **<font color='green'>Summary</font>**
- **Autonomous Exploration**: UAV decides where to go next based on unknown areas
- **Real-time Mapping**: Builds occupancy map from LiDAR data
- **View Planning**: Intelligently selects viewpoints that maximize information gain
- **Integrated Pipeline**: From sensing → planning → trajectory execution

## Data Flow
```
LiDAR Points → NBV Planner (decides where to look next)
                    ↓
         Exploration Node (plans trajectory to target)
                    ↓
         position_hold/trajectory (for tracker to follow)
                    ↓
              UAV flies to next view
                    ↓
         (cycle repeats with new sensor data)
```

## Stack

This is the **highest level** in your system hierarchy:
1. **Explorations**: Decides WHERE to explore (next best view)
2. **Planner**: Plans HOW to get there (global path + TOPP)
3. **Tracker**: Executes the trajectory (low-level control)
4. **Executor**: Manages state machine during flight

## Relationship to Previous Launch File
Together they form a complete autonomous navigation system:
- Planner: Plans the path (using octomap for obstacle avoidance)
----------------------------------------
# DUMP THEM ALL

This is the **third piece of the puzzle**:

1. **Global Planner** (from previous file): Plans the path using octomap
2. **TOPP Generator** (in both files): Creates time-optimal trajectory
3. **Execute Trajectory** (this file): Actually flies the trajectory

- Trajectory Tracker: Tracks the path (using carrot following algorithm)


An autonomous exploration or inspection system (NBVP = Next Best View Planning) where the UAV plans and executes trajectories to inspect unknown environments.
---
---
