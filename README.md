# px4_offboard_custom_controller

A basic ROS-based flight controller for PX4 compatible drones. After the drone reaches the target takeoff altitude (hover point), the script now calls the MAVROS set_mode service with custom_mode="AUTO.LOITER" to engage PX4’s Loiter (Hold) mode. This leverages the flight controller’s built-in position hold capability instead of requiring our script to constantly send setpoints. (PX4 uses the string "AUTO.LOITER" for Loiter mode​, similar to how "AUTO.LAND" is used for Land mode.) Once LOITER mode is active, the autopilot itself keeps the drone hovering at the last position, greatly reducing the need for continuous outbound commands and improving robustness in case of communication delays or drops.

## Modes of control
- Velocity mode
- Poisition mode
- Hold flight mode (aka LOITER)

## Teleoperation
- Keyboard (press r for landing)
Moving around:
   u    i    o
   j    k    l
   m    ,    .
t : up (+z)
b : down (-z)

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