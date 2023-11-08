# robot_chase

This program allows the user control the red robot, while the blue robot will automatically try to chase the other!

https://github.com/Andy-Leo10/robot_chase/assets/60716487/ce4d4d2c-ee8b-451f-9696-3128707ae1c1

## Mandatory
+ Download the package of the robot and compile it
```
git clone https://github.com/Andy-Leo10/barista_robot_description.git
cd ~/ros2_ws/ ;colcon build --packages-select barista_robot_description; source install/setup.bash
```
+ Download this package and compile it
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_chase; source install/setup.bash
```

## Launch files
- [x] Start simulation with 2 robot-baristas
```
ros2 launch barista_robot_description barista_two_robots.launch.py
```
- [x] Run the hunter (blue robot)
```
ros2 run robot_chase robot_chase
```
- [x] Run away by keyboard input (red robot)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel
```


