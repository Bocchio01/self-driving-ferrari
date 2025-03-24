# Software

This directory contains the software for the autonomous RC ferrari.
It's itself a ROS workspace, with the following packages:

- `rc_vehicle_localization`: using external knowledge of the environment to localize the vehicle
- `rc_vehicle_path_planning`: planning the path to follow
- `rc_vehicle_motion_control`: controlling the vehicle to follow the planned path
- `rc_vehicle_perception`: detecting obstacles and other vehicles
- `rc_vehicle_remote`: remote control of the vehicle

![See-Think-Act](.resources/See-Think-Act%20cycle.jpg)

https://github.com/nasa-jpl/osr-rover-code/blob/foxy-devel/setup/rpi.md