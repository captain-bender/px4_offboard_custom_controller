# px4_offboard_custom_controller

A basic ROS-based flight controller for PX4 compatible drones.

## Modes of control
- Velocity mode
- Poisition mode

## Teleoperation
- Keyboard (press r for landing)
- Joystick (to-do)

## Execution
```
roslaunch px4_offboard_custom_controller bringup_the_simulation.launch
rosrun px4_offboard_custom_controller offboard_controller.py
rosrun px4_offboard_custom_controller teleop_twist_keyboard.py
```

## Setup
- Ubuntu 20.04.6 LTS
- ROS Noetic

### Author (or who to blame)
Angelos Plastropoulos