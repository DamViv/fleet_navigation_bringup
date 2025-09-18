# Fleet Navigation Bringup

## Overview

The `fleet_navigation_bringup` package provides a comprehensive launch system for multi-robot navigation using ROS 2 Jazzy. It integrates Nav2 navigation stack with traversability mapping and custom goal remapping for coordinated fleet operations.

## Features

- **Multi-robot Nav2 integration**: Launches Nav2 navigation stack for multiple robots with individual namespaces
- **Traversability mapping**: Integrates terrain analysis for safe navigation in complex environments  
- **Goal remapping**: Custom node for coordinating navigation goals across the fleet
- **Modular configuration**: Individual parameter files for each robot
- **Scalable architecture**: Easy to add/remove robots from the fleet

## Package Structure

```
fleet_navigation_bringup/
├── launch/
│   ├── fleet_navigation_bringup.launch.py    # Main launch file
│   └── custom_nav2_bringup.launch.py         # Nav2 configuration with namespaces
│   └── custom_navigation_launch.py           # Launch needed Nav2 node
├── params/
│   ├── fleet_0_nav2_params.yaml              # Nav2 params for robot 0
│   ├── fleet_1_nav2_params.yaml              # Nav2 params for robot 1
│   └── ...
├── config/
│   ├── fleet_0_config.yaml                   # Robot 0 configuration
│   ├── fleet_1_config.yaml                   # Robot 1 configuration
│   └── ...
├── fleet_navigation_bringup/
│   ├── __init__.py
│   └── goto_remaps.py                        # Goal remapping node for namesapces
└── README.md
```

## Dependencies

- ROS 2 Jazzy
- Nav2 navigation stack
- `traversability_map` package
- Standard ROS 2 geometry and navigation messages

## Installation

1. Clone this package into your ROS 2 workspace:
```bash
cd ~/ros_jazzy_ws/src/navigation/
git clone <repository_url> fleet_navigation_bringup
```

2. Build the package:
```bash
colcon build --packages-select fleet_navigation_bringup
source install/setup.bash
```

## Usage

### Basic Launch

Launch the complete fleet navigation system:

```bash
ros2 launch fleet_navigation_bringup fleet_navigation_bringup.launch.py
```

### Robot Namespaces

The system automatically creates namespaced robots:
- Robot 0: `/fleet_0`
- Robot 1: `/fleet_1`
- ...

### Navigation Topics

Each robot exposes standard Nav2 topics under its namespace:

**Robot 0 topics:**
- `/fleet_0/navigate_to_pose` - Navigation action server
- `/fleet_0/goal_pose` - Goal pose input
- `/fleet_0/cmd_vel` - Velocity commands
- `/fleet_0/map` - Occupancy grid map
- `/fleet_0/local_costmap/costmap` - Local costmap
- `/fleet_0/global_costmap/costmap` - Global costmap


### Robot-Specific Configuration

Each robot has individual configuration files:

**Navigation parameters** (`params/fleet_X_nav2_params.yaml`):
- Controller settings
- Planner configuration
- Costmap parameters
- Behavior tree settings

**Robot configuration** (`config/fleet_X_config.yaml`):

Used for traversability and nav2 config
```yaml
pc_topic: "/fleet_X/points"
imu_in: "/fleet_X/imu/data"
imu_out: "/fleet_X/imu/data_filtered"
security_distance: 0.5
max_slope: 0.5
ground_clearance: 0.1
robot_height: 1.0
robot_width: 0.6
robot_length: 0.8
```

## Nodes

### goto_remaps Node

Custom node for remapping navigation goals across the fleet.

**Subscribed Topics:**
- `goal_pose` (geometry_msgs/PoseStamped) - Input goal pose

**Published Topics:**
- `navigate_to_pose/goal` (geometry_msgs/PoseStamped) - Remapped goal for Nav2

**Parameters:**
- `robot_id` (int) - Robot identifier
- `use_sim_time` (bool) - Use simulation time


## License

This package is licensed under the MIT License.

## Maintainer

**Damien Vivet**  
Email: damien.vivet@isae.fr  
Institution: ISAE-SUPAERO

## Version History

- **v1.0.0**: Initial release with multi-robot Nav2 integration
- **v1.1.0**: Added traversability mapping support
- **v1.2.0**: Integrated goal remapping functionality