#!/bin/sh
set -e

cd ~/catkin_ws
catkin build msckf_vio
source ~/catkin_ws/devel/setup.bash

# ./devel/lib/msckf_vio/msckf_vio_tests --silence-stdcout

# roslaunch msckf_vio image_processor_triclops.launch
roslaunch msckf_vio msckf_vio_triclops.launch
# roslaunch msckf_vio msckf_vio_euroc.launch
# roslaunch msckf_vio msckf_vio_fla.launch
