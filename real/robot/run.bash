#!/bin/bash

source /opt/ros/humble/setup.bash

cd workspace
source install/setup.bash

ros2 run tb3_controller tb3_controller_node --ros-args -p act_mode:=real

