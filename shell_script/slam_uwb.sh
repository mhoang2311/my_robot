#!/bin/bash

#gnome-terminal --tab -- rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_debug_topics:=true
#gnome-terminal --tab -- rostopic pub -1 /agv/start_imu std_msgs/Bool true
#sleep 5
gnome-terminal --tab -- python3 ~/catkin_ws/src/my_new_robot/my_robot_control/scripts/ekf_uwb_position.py
gnome-terminal --tab -- python3 ~/catkin_ws/src/my_new_robot/my_robot_control/scripts/ekf_uwb_position.py
sleep 15
gnome-terminal --tab -- roslaunch ~/catkin_ws/src/my_new_robot/my_robot_slam_new/launch/my_agv_ekf_uwb_karto.launch



