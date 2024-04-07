#!/bin/bash

# mros2proxy
cp ./digital/config/custom.json third-party/hakoniwa-ros2pdu/config/
cp ./digital/config/ros_msgs.txt third-party/hakoniwa-ros2pdu/config/
cp digital/fix-codes/mros2/templates_generator.py third-party/hakoniwa-ros2pdu/mros2-posix/mros2/mros2_header_generator/templates_generator.py

