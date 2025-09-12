
# ros2_control Configuration Guide

## Overview

ros2_control is a ROS 2 framework for robot control that implements PID controllers and manages hardware interfaces. It provides a standardized way to control robots with various hardware configurations

## Installation

``` bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## Learning resource

To understand how ros2_control works, watch this comprehensive course:
[Ros2 Control Course](https://youtu.be/B9SbYjQSBY8?si=dmxxUANcyLlxqE-z) - Estimated time: 2 hours, but it is definitely worth it

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

Dacă Arduino nu este detectat pe `/dev/ttyACM0`, verificați portul corect folosind comanda de mai jos și actualizați parametrul `device` în consecință.

## Explanation of PID Parameters in ros2_control Configuration - from official ros2_control source 

The parameters listed in the URDF configuration file are related to the PID (Proportional-Integral-Derivative) controller used for motor control in the `ros2_control` framework. These parameters tune the behavior of the controller to achieve precise and stable movement of the robot's wheels. Below is an explanation of each parameter:

- **`<param name="pid_p">100</param>`**:
  - **Proportional Gain (P)**: This parameter determines the reaction to the current error (the difference between the desired and actual velocity or position). A higher value makes the system respond more aggressively to errors, but if set too high, it can cause overshooting or oscillations. A value of `100` indicates a strong proportional response to correct errors quickly.

- **`<param name="pid_d">30</param>`**:
  - **Derivative Gain (D)**: This parameter controls the reaction to the rate of change of the error. It helps to dampen the system and reduce overshooting by predicting future error based on its current rate of change. A value of `30` suggests a moderate damping effect to stabilize the response and prevent rapid oscillations.

- **`<param name="pid_i">0</param>`**:
  - **Integral Gain (I)**: This parameter addresses the accumulated error over time, helping to eliminate steady-state errors (e.g., when the robot is consistently slightly off the target). A value of `0` means the integral component is disabled, likely to avoid issues like windup (where accumulated error causes overcorrection) or because the system doesn't require correction for persistent small errors.

- **`<param name="pid_o">60</param>`**:
  - **Output Limit (O)**: This parameter sets the maximum output value that the PID controller can send to the motor (likely related to PWM or speed command). A value of `60` limits the controller's output to prevent excessive speed or torque that could damage the hardware or cause instability. It acts as a safety cap on the control signal.