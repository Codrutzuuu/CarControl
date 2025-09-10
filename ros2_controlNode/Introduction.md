
# ros2_control Configuration Guide

## Overview

ros2_control is a ROS 2 framework for robot control that implements PID controllers and manages hardware interfaces. It provides a standardized way to control robots with various hardware configurations

## Installation

``` bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## Learning resource

To understand how ros2_control works, watch this comprehensive course:
[Ros2 Control Course](https://youtu.be/B9SbYjQSBY8?si=dmxxUANcyLlxqE-z) - required time - 2 hours, but it definetly worth it

## Architecture Overview

![image](/ros2_controlNode/img/image.png)
-----

## Arduino Integration - ros2_arduino_bridge

To interface ros2_control with Arduino, use the ros2_arduino_bridge located in:
**diffdrive_arduino/hardware**

## Key Components

1. Hardware Interface - Communicates with Arduino firmware

2. Controller Manager - Manages multiple controllers

3. PID Controllers - For velocity/position control

4. Joint State Broadcaster - Publishes joint states

## Controller Configuration

[Here you can find the configuration file](/ros2_controlNode/diffbot_controllers.yaml)

## Urddf Configuration

In this urdf file, you have the configuration of the hardware interface that allows you to communicate to arduino>

``` xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="diffbot_ros2_control" params="name prefix">

    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>diffdrive_arduino/DiffDriveArduinoHardware</plugin>
        <param name="left_wheel_name">left_wheel_joint</param>
        <param name="right_wheel_name">right_wheel_joint</param>
        <param name="loop_rate">30</param>
        <param name="device">/dev/ttyACM0</param>
        <param name="baud_rate">57600</param>
        <param name="timeout_ms">1000</param>
        <param name="enc_counts_per_rev">94</param>
        <param name="pid_p">100</param>
        <param name="pid_d">30</param>
        <param name="pid_i">0</param>
        <param name="pid_o">60</param>
      </hardware>
      <joint name="${prefix}left_wheel_joint">
        <command_interface name="velocity"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="${prefix}right_wheel_joint">
        <command_interface name="velocity"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </ros2_control>

  </xacro:macro>

</robot>
```

- loop_rate and baud_rate must be the same as Arduino Firmare, otherwise it won't work
- enc_count_per_rev - it represents the number of ticks that motor does for a full revolution - this is provided by Hall sensor and magnets 

### !! ACM0 - this is the Arduino port, if you use a fake arduino, you have to change the port name, just inspect with:

``` bash
ls /dev/tty*
```