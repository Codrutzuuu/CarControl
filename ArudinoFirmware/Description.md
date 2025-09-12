# Arduino Code Explanation for Robot Motor Control and Encoder Handling

This document explains the provided Arduino code, which is designed to control a differential drive robot using motor control and encoder feedback. The code handles motor direction, speed, braking, and processes serial commands for remote control.

## Overview

The code is written for an Arduino board to control two motors (left and right) for a differential drive robot. It uses encoder feedback to track wheel movement and accepts serial commands to set motor speeds or retrieve encoder data. The code also implements a braking mechanism and smooth speed transitions.

## Pin Definitions and Constants

- **Motor Control Pins**:
  - `MOTOR_DIR_A (8)` and `MOTOR_DIR_B (7)`: Control the direction of the motors.
  - `MOTOR_PWM_R (5)` and `MOTOR_PWM_L (9)`: PWM pins to control the speed of the right and left motors, respectively.
  - `MOTOR_BRAKE (6)`: Controls the brake mechanism for the motors.

- **Encoder Pins**:
  - `ENCODER_R (2)` and `ENCODER_L (3)`: Pins connected to the right and left wheel encoders for tracking movement.

- **Constants**:
  - `SERIAL_TIMEOUT (1000)`: Timeout for serial communication in milliseconds.
  - `COMMAND_END ("\r\n")`: String used to terminate serial commands.
  - `MAX_PWM (40)`: Maximum PWM value for motor speed (limits speed to avoid damage).
  - `PWM_STEP (2)`: Incremental step for smooth speed changes.
  - `BRAKE_DELAY (300)`: Delay in milliseconds before applying the brake if no activity is detected.

## Global Variables

- **Encoder Counters**:
  - `encoderR` and `encoderL`: Track the position of the right and left wheels based on encoder pulses.
  - `lastTimeR` and `lastTimeL`: Store the last time an encoder pulse was detected to debounce the signal.

- **Motor Speed Variables**:
  - `targetR` and `targetL`: Desired speed for the right and left motors (received via serial commands).
  - `currentR` and `currentL`: Current speed of the right and left motors (gradually adjusted to match target).

- **Timing Variables**:
  - `lastActivity`: Tracks the last time a motor command was executed (used for braking).
  - `lastSend`: Tracks the last time encoder data was sent over serial.

## Functions

### `handleEncoderR()` and `handleEncoderL()`

These are interrupt service routines (ISRs) triggered by changes on the encoder pins. They update the encoder counters (`encoderR` and `encoderL`) based on the motor direction (determined by `MOTOR_DIR_A` and `MOTOR_DIR_B`).

- A debounce mechanism is implemented using a 500-microsecond delay to ignore noise or rapid changes.
- The direction of the motor determines whether the encoder count increments or decrements.

### `setMotorSpeed(int speedR, int speedL)`

This function sets the speed and direction of the motors.

- **Speed Constraint**: Limits the input speeds to the range `[-MAX_PWM, MAX_PWM]`.
- **Direction Control**: Based on the sign of `speedR` and `speedL`, the function sets the motor direction:
  - Both positive: Move forward (`MOTOR_DIR_A HIGH`, `MOTOR_DIR_B LOW`).
  - Both negative: Move backward (`MOTOR_DIR_A LOW`, `MOTOR_DIR_B HIGH`).
  - Right negative, Left positive: Turn right (`MOTOR_DIR_A HIGH`, `MOTOR_DIR_B HIGH`).
  - Right positive, Left negative: Turn left (`MOTOR_DIR_A LOW`, `MOTOR_DIR_B LOW`).
- **PWM Output**: Writes the absolute speed value to the PWM pins for each motor.
- **Brake Control**: If both speeds are 0, the brake is engaged (`MOTOR_BRAKE HIGH`); otherwise, it is disengaged, and `lastActivity` is updated.

### `processCommand(String cmd)`

This function processes incoming serial commands to control the robot or retrieve data.

- **Command "m <speedR> <speedL>"**: Sets the target speeds for the right and left motors (scaled by 0.7 to reduce speed). Responds with "ACK".
- **Command "e"**: Returns the current encoder values for the right and left wheels in the format `e <encoderR> <encoderL>`.
- **Command "u <value>"**: Placeholder for PID tuning or other updates (responds with "PID_SET").
- **Unknown Commands**: Responds with "UNKNOWN_CMD".

All responses are terminated with `COMMAND_END` (`\r\n`).

## Setup and Loop

### `setup()`

- Initializes the motor control and encoder pins as outputs and inputs, respectively.
- Sets the brake to engaged (`HIGH`) initially.
- Attaches interrupts to the encoder pins to call `handleEncoderR()` and `handleEncoderL()` on state changes.
- Starts serial communication at 57600 baud and waits for the connection.
- Sends a "READY" message to indicate the system is initialized.

### `loop()`

- **Serial Input Handling**: Reads incoming serial data until a `\r` character is received and processes the command using `processCommand()`.
- **Smooth Speed Adjustment**: Gradually adjusts `currentR` and `currentL` toward `targetR` and `targetL` using `PWM_STEP` for smooth transitions.
- **Motor Control**: Calls `setMotorSpeed(currentR, currentL)` to update motor speeds.
- **Periodic Encoder Update**: Every 50 milliseconds, sends the current encoder values over serial.
- **Brake Timeout**: If no activity is detected for `BRAKE_DELAY` milliseconds, engages the brake and resets all speed values to 0.

### **Important**

Arduino recieves ONLY the PWM from Jetson <PWM, PWM>
and it sends encoders values e encL encR - otherwise it won't be compatible with ros2_cotrol node.

### How to use it?

Simply load the code on Arduino...fleșuiești placa vericule cu codul tau and it s done

## Summary

This Arduino code provides a robust framework for controlling a differential drive robot. It handles motor speed and direction, tracks wheel movement using encoders, processes serial commands for remote operation, and implements safety features like braking after inactivity. The code is designed for real-time control with smooth speed transitions and noise filtering on encoder inputs.
