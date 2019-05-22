#!/bin/bash

cd `dirname $0`

echo "*** Source ROS"
source /opt/ros/kinetic/setup.sh
echo "*** Remove ./build/ and ./devel/"
rm -rf ./build ./devel
echo "*** Run catkin_make"
catkin_make
echo "*** Source workspace"
source ./devel/setup.sh
cd ./devel/
develpath=`pwd`
cd ../build/
echo "*** Set devel path to $develpath"
cmake ../src/ -DCATKIN_DEVEL_PREFIX="$develpath"
echo "*** Add ORB_SLAM2 to ros path"
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/hustac/ORB_SLAM2/Examples/ROS
echo "*** Run qtcreator"
cd ../
~/Qt*/Tools/QtCreator/bin/qtcreator 1>/dev/null 2>&1 &

