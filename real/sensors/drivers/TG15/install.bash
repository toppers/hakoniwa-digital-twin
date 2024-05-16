#!/bin/bash

THIRD_PARTY_PATH=../../../../third-party

cp ydlidar_ros_driver/launch/* ${THIRD_PARTY_PATH}/ydlidar_ros2_driver/launch/
cp ydlidar_ros_driver/param/* ${THIRD_PARTY_PATH}/ydlidar_ros2_driver/param/
cp -rp ${THIRD_PARTY_PATH}/ydlidar_ros2_driver ../../workspace/src/
