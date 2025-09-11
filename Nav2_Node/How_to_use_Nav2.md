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
![Image](/Nav2_Node/img/LocalMap.png)
or something similar

Basically, what you see there is the local_costmap.

## What is a costmap?

Costmaps are 2D grid-based representations of the environment used in navigation where each cell contains a cost value indicating how desirable or dangerous that area is for the robot to traverse.

Yeah, in Nav2, we have 2 important concepts:

## Global Costmap

### Purpose

The global costmap provides a world-wide, persistent representation of the environment used for long-term path planning.

### Key Characteristics

- Static representation of the known environment

- Covers the entire mapped area

- Built from the static map (from SLAM) and persistent obstacles

- Used for global path planning (calculating routes from start to goal)

## Local Costmap

The local costmap provides a short-term, rolling window representation used for local obstacle avoidance and immediate navigation decisions.

### Characteristics

- Dynamic and frequently updated (5-10Hz)

- Covers only the immediate surroundings of the robot

- Includes temporary obstacles and sensor noise

- Used for local planning and obstacle avoidance

### !!! Diferenta majora o sa vedeti ca una va fi de folos ca sa detectati obstacole imediate, precum oameni(local costmap), iar cealala va fi o harta mare a tuturor imprejurimilor

## How to run the robot

1. Estimate the position
   ![Estimate](/Nav2_Node/img/Estimate.png)
2. Send a goal
   ![Send Goal](/Nav2_Node/img/SendGoal.png)
3. You wil see a trajectory