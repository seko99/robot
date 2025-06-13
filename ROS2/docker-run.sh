#!/bin/bash
docker run --device /dev/ttyUSB0:/dev/ttyUSB0 -e ROS_LOCALHOST_ONLY=0 -e ROS_DOMAIN_ID=0 --name robot-teleop robot-teleop
docker run -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 --device /dev/ttyS5:/dev/ttyS5 --network host --name robot-lidar robot-lidar
docker run -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 --device /dev/ttyUSB0:/dev/ttyUSB0 --network host --name robot-odometry robot-odometry
docker run -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 --device /dev/ttyUSB1:/dev/ttyUSB1 --network host --name robot-sonar robot-sonar
