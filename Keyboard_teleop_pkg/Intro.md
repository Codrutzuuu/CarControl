# What is Keyboard_teleop_pkg?

As you may have noticed, there is another package called `keyboard_teleop_pkg`. This represents the first attempt at creating our autonomous robot. 😊

## What Do You Have Here?

1. **Configuration Files**: All the necessary configuration files are included, such as EKF, SLAM Toolbox, and Nav2.
2. **Odom Bridge**: This component takes information from the `/odometry/filtered` topic and republishes it on the `/odom` topic.
3. **IMPORTANT!!! CmdVelBridge**: This bridge takes data from the `/cmd_vel` topic and publishes it to `/diffbot_base_controller/cmd_vel_unstamped`. This is a crucial step because `ros2_control` only reads information from the `/diffbot_base_controller/cmd_vel_unstamped` topic, while Nav2 posts on `/cmd_vel`.

## And the Most Important Thing

You have a WASD control for our toy car. in order to run it, just run the nodes from [here buddy](/Nav2_Node/How_to_use_Nav2.md), and in an another terminal run:

``` bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_teleop_pkg keyboard_teleop 
```

and now you should be able to move the robot with WASD like in GTA 6, sper ca in ziua in care citesti tu asta, GTA 6 sa apara

## IMPORTANT STUFF

You must build the packages whenever you make a modification in files.

But do not build all the packages everytime with: `colcon build`, use instead `colcon build --packages-select your_package_name1 your_package_name2` you got the point

If you wanna build all, be careful...because Jetson can crash for lacking of memory - use `colcon build --parallel-workers 4` in order to limit the number of packages build simultaneously at 4.
