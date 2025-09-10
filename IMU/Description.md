# IMU Sensor configuration

## Connection

- GND: Conectează pinul GND al IMU la un pin GND de pe Jetson.
- VCC: Conectează pinul VCC al IMU la pinul de 3.3V de pe Jetson
- SDA: Conectează pinul SDA al IMU la pinul 3 (SDA, I2C) de pe Jetson.
- SCL: Conectează pinul SCL al IMU la pinul 5 (SCL, I2C) de pe Jetson

Check if it s connected properly with the following command:
``` bash
i2cdetect -y -r 7 
```
it should appear 68 