# Extended Kalman Filter (EKF) Configuration Guide

## Overview

The Extended Kalman Filter (EKF) node from the robot_localization package is used for sensor fusion, combining odometry and IMU data to produce a more accurate robot pose estimation.

## Installation

The EKF is part of the robot_localization package:

```bash

sudo apt install ros-humble-robot-localization
```

## Purpose

The EKF fuses multiple sensor inputs to:

- Reduce noise from individual sensors

- Minimize drift in position and orientation estimates

- Provide a smoothed, continuous pose estimate

## Data Sources

In our implementation, we fuse two primary data sources:

- Wheel Encoders from **/diffbot_base_controller/odom** - topic
- IMU Sensor from **/imu/data** topic

## Output topics

The EKF node publishes the following topics

- /odometry/filtered - Fused odometry data

- /tf - Transform between odom and base_link frames

## Configuration file

[Here you have the configuration file for EKF](/EKF%20Node/ekf_config.yaml)