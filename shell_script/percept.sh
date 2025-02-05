#!/bin/bash

source ~/catkin_ws/devel/setup.bash 
#rosrun final_result final_result 
roslaunch darknet_ros yolo_v4-tiny.launch
