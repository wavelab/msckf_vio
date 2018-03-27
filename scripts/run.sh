#!/bin/sh

cd ~/catkin_ws
catkin build
source ~/catkin_ws/devel/setup.bash

# roslaunch msckf_vio image_processor_triclops.launch
roslaunch msckf_vio msckf_vio_triclops.launch
# roslaunch msckf_vio msckf_vio_euroc.launch
