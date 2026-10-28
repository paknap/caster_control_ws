# Caster control workspace

Minimal ROS 2 workspace for the Caster robot.

| Package | Purpose |
| --- | --- |
| `odrive_ros` | USB ODrive driver, odometry, and motor-control services |
| `robot_description` | Caster Xacro, meshes, optional sensor visuals, and state publishers |
| `teleop_launch` | Joystick configuration and driver/teleop bringup |

## Launch

Change current directory into workspace root folder or modify paths in the following commands.

Source `install/setup.bash` and your middleware environment in each terminal.

### Joystick teleoperation + motor drivers

```sh
./src/teleop_launch/scripts/launch_teleop.sh
```
Joystick can be configured in [joy.yaml](src/teleop_launch/params/joy.yaml).

### Motor drivers alone
```sh
ros2 launch odrive_ros odrive.launch.xml
```

### Robot model
```sh
ros2 launch robot_description description.launch.py
```

### RViz
```sh
ros2 run rviz2 rviz2 -d caster.rviz
```