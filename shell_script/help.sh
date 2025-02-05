#!/bin/bash

source ~/catkin_ws/devel/setup.bash 
sleep 8
rosrun rosserial_python serial_node.py port:=/dev/ttyUSB0 _baud:=57600 &
#rosrun processing temp.py 
#rosrun final_result final_result 
#roslaunch darknet_ros yolo_v4-tiny.launch 
