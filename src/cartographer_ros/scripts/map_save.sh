#!/bin/bash
rosservice call /finish_trajectory 0
sleep 5
rosservice call /write_state "{filename: '${HOME}/Documents/slam/cartographer_comments-yg/src/cartographer_ros/maps/$1.pbstream'}"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Documents/slam/cartographer_comments-yg/src/cartographer_ros/maps/$1 -pbstream_filename=${HOME}/Documents/slam/cartographer_comments-yg/src/cartographer_ros/maps/$1.pbstream  -resolution=0.05
echo "succeed save map!"
