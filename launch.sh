#!/bin/bash

launch_path="/home/infantry5/imca_hero2025"
cd $launch_path 


launchCommand(){
	. /opt/ros/humble/setup.sh 
   	colcon build --symlink-install --parallel-workers 3
	source install/setup.bash 
	ros2 launch rm_bringup bringup.launch.py
}

while true; do
	launchCommand
	sleep 5
done



