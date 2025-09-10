# Hardware Setup

## Arduino Connections

The robot uses an Arduino to control two DC motors via a motor driver and read wheel encoder signals. The pinout is as follows:

### Left Motor

- PWM: Pin 9 (LEFT_MOTOR_PWM)
- Direction: Pin 7 (LEFT_DIR_PIN)
- Brake: Pin 6 (BRAKE_PIN)
- Left Encoder: Pin 2 (LEFT_ENCODER_A)
  
### Right Motor

- PWM: Pin 5 (RIGHT_MOTOR_PWM)
- Direction: Pin 8 (RIGHT_DIR_PIN)
- Brake: Pin 6 (BRAKE_PIN)
- Right Encoder: Pin 3 (RIGHT_ENCODER_A)
  
The encoders use single-channel (A) signals with interrupt-driven counting, incrementing or decrementing based on motor direction (1 for forward, 0 for backward). The robot parameters are:

- Wheel radius: 0.14 m
- Track width: 0.68 m
- Encoder resolution: 96 ticks per revolution
