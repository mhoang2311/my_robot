#!/bin/bash

gnome-terminal --tab -- roslaunch ~/catkin_ws/src/my_new_robot/my_robot_control/bringup/sensors_bringup.launch
gnome-terminal --tab -- roslaunch ~/catkin_ws/src/my_new_robot/my_robot_control/bringup/uwb1.launch
gnome-terminal --tab -- roslaunch ~/catkin_ws/src/my_new_robot/my_robot_control/bringup/uwb2.launch
sleep 15
gnome-terminal --tab -- python3 ~/catkin_ws/src/my_new_robot/my_robot_control/scripts/uwb_position.py


