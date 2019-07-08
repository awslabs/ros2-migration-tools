#!/usr/bin/env bash

current_dir=$(pwd)
SRC_PATH_TO_UPGRADE=$(python3 utilities.py 2>&1)
echo "src path: "
echo $SRC_PATH_TO_UPGRADE
cd $SRC_PATH_TO_UPGRADE

#cd ..
#cmake ./src -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source /opt/ros/kinetic/setup.bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON


cd $current_dir
python3 ros_upgrader.py $SRC_PATH_TO_UPGRADE