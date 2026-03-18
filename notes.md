# **<font color='green'>Arm & Takeoff</font>** 
   
   - "#roslaunch uav_ros_control pid_carrot.launch manual_takeoff:=false"
        - "#rosrun ardupilot_gazebo automatic_takeoff.sh 0.7"
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

## Node 3. **Exploration Node** (`interface_nbvp_rotors/exploration`)
Manages the exploration mission:
- Receives NBV planner's suggestions
- Generates trajectories to the next best view
- **Output**: Publishes trajectory points to `position_hold/trajectory`[`trajectory_msgs/MultiDOFJointTrajectoryPoint`]

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

## Node 2. **Global Planner Node** (`larics_motion_planning/global_planner_node`)
This is the key component that **supplies the path**:
- **Purpose**: Plans global paths through the environment
- **Input**: Listens to `/octomap_binary` (3D occupancy map of the environment)
- **Configuration**: Loads settings from `kopterworx_config.yaml`
- **Function**: Generates collision-free paths from start to goal positions

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
