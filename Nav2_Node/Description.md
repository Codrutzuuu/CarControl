# Nav2: Navigation Framework for ROS 2 (that part of documentation is from the official Navgigation2 documentation)

## Overview

Nav2 is the next-generation navigation stack for the Robot Operating System 2 (ROS 2). It is a highly modular and extensible framework designed to enable robots to navigate autonomously in both indoor and outdoor environments. Nav2 builds upon the concepts of the original ROS 1 navigation stack but introduces significant improvements in terms of modularity, performance, and support for ROS 2's modern architecture.

## Key Features

- **Modular Architecture**: Nav2 is split into multiple components (e.g., global planner, local planner, costmap, recovery behaviors) that can be customized or replaced based on specific robot requirements.
- **ROS 2 Integration**: Fully compatible with ROS 2, leveraging its communication mechanisms like DDS for real-time performance and scalability.
- **Dynamic Obstacle Avoidance**: Uses sensor data (e.g., LiDAR, cameras) to detect and avoid obstacles in real-time through local planning.
- **Global Path Planning**: Computes optimal paths from a starting point to a goal using algorithms like A* or Dijkstra's algorithm.
- **Costmaps**: Maintains layered costmaps (static, obstacle, inflation) to represent the environment and guide navigation decisions.
- **Behavior Trees**: Utilizes behavior trees for managing complex navigation tasks and recovery behaviors when the robot gets stuck.
- **Multi-Robot Support**: Designed to handle navigation for multiple robots in a shared environment. 

## Core Components

1. **Global Planner**: Plans a high-level path from the robot's current position to the goal, ignoring dynamic obstacles.
2. **Local Planner**: Generates velocity commands to follow the global path while avoiding dynamic obstacles.
3. **Costmap 2D**: Represents the environment as a 2D grid with costs associated with traversal (e.g., free space, obstacles, inflation zones).
4. **Recovery Behaviors**: Handles situations where the robot is stuck or unable to proceed, such as rotating in place or backing up.
5. **Controller Server**: Manages the local planner and computes velocity commands for the robot.
6. **Planner Server**: Manages the global planner and provides path planning services.
7. **Behavior Server**: Orchestrates high-level navigation tasks using behavior trees.

## Workflow
1. **Initialization**: Load maps (static or dynamic) and configure parameters for planners and costmaps.
2. **Goal Setting**: A user or higher-level system sends a navigation goal (e.g., a specific pose in the map).
3. **Path Planning**: The global planner computes a feasible path to the goal.
4. **Path Following**: The local planner generates velocity commands to follow the path while avoiding obstacles.
5. **Recovery (if needed)**: If the robot cannot proceed, recovery behaviors are triggered to resolve the issue.
6. **Goal Reached**: The robot stops once it reaches the goal within a specified tolerance.

inners due to the modular and configurable nature.

## Getting Started

To get started with Nav2 in ROS 2, follow these steps:

1. Install ROS 2  on your system.
2. Install the Nav2 package using `sudo apt install ros-humble-navigation2`.
3. Configure your robot's URDF and sensor setup in ROS 2.
4. Launch the Nav2 stack using provided launch files or create a custom one.
5. Use RViz or other tools to set navigation goals and visualize the robot's path.

## Resources

- Official Nav2 Documentation: [https://docs.nav2.org/](https://docs.nav2.org)
- ROS 2 Tutorials: [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
- GitHub Repository: [https://github.com/ros-planning/navigation2](https://github.com/ros-planning/navigation2)

## Configuration file

[Here you have the configuration file](/control/keyboard_teleop_pkg/config/nav2_params.yaml)

### !!! Please be careful, even the smallest mistake in configuration, can be fatal 

## Conclusion

Nav2 is a powerful and flexible navigation framework for ROS 2, enabling robots to perform autonomous navigation tasks with high reliability. Its modular design and integration with ROS 2 make it a cornerstone for modern robotic applications, though it requires careful configuration and tuning to achieve optimal performance.
