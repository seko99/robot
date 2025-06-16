#!/bin/bash

source /opt/ros/jazzy/setup.bash

ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/image_raw &
ros2 run rviz2 rviz2 &