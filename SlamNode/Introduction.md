
# Overview

  **Slam Toolbox** is a ROS 2 package for Simultaneous Localization and Mapping (SLAM). This document covers the setup and configuration for use with a differential drive robot.

**SLAM** big package used for mapping and localization for SLAM, and for our differential robot it needs the following configuration as in the configuration file:  **ros2_ws/src/keyboard_teleop_pkg/config/slam_toolbox.yaml**

## Installation

```bash
sudo apt install ros-humble-slam-toolbox 
```

## Official documentation

Here you can find out all the details for every single parameter [Link here to SLAM](https://docs.ros.org/en/humble/p/slam_toolbox)

## Key Reminder

Without LiDAR, you cannot generate a map.(obvious, duh), and sometimes the lidar could have some critical errors such as:

- **SL_RESULT_OPERATION_TIMEOUT** - do not panic(or you should 😊) – a feedback cable on the Lidar is detached, and you must go to Sorin to help you repair it
- **Error 8000xxxx** - that means you are using the wrong port, in that case you have to see on witch port is your lidar mapped:

```bash
ls /dev/ttyUSB*
```

with that command you should see all the devices available, even the lidar

## Configuration file 

[Here you have the configuration file for SLAM](/SlamNode/slam_toolbox_config.yaml)
