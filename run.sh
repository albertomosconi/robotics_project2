#!/bin/sh

cd ../..;
catkin_make;
[ $? -ne 0 ] && exit 1;
roslaunch project2 mapping.launch;