# How to use Nav2 - make the robot to work

## Launch the robot

To launch all the nodes use the commands:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch diffdrive_arduino diffbot.launch.py use_sim_time:=false 
```
[Here you have all the launch file](/src/diffdrive_arduino/bringup/launch/diffbot.launch.py)

## What you will see in Rviz

After launching all the nodes you will see this:

## How to run the robot 

1. Estimate the position
2. Send a goal 
3. You wil see a trajectory