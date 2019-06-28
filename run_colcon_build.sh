#!/bin/bash

current_dir=$(pwd)
ROS2_SRC_PATH=$(python3 utils.py 2>&1)
cd $ROS2_SRC_PATH
cd ..
cmake ./src -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cd $current_dir

python3 ros_upgrader.py $ROS2_SRC_PATH