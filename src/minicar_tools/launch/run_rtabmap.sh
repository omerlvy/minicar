#!/bin/bash
#source /etc/profile.d/ros.sh
roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start" rtabmapviz:=false depth_registration:=true
