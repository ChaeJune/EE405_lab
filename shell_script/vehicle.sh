#!/bin/bash

source ~/catkin_ws/devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch &
sleep 8
#rosrun rosserial_python serial_node.py port:=/dev/ttyUSB0 _baud:=57600 &
#(rosrun part is not executable then cd ~/arduino-1.8.15/libraries/)
roslaunch pointcloud_projection depth_image_projection.launch &
sleep 3
roslaunch local_costmap_generator run.launch &
rosrun final_result final_result & 
roslaunch motion_primitives_planner run.launch 
#rosrun final_result final_result &
#roslaunch darknet_ros yolo_v4-tiny.launch 
#roslun processing temp.py
