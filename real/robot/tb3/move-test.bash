#!/bin/bash

if [ $# -ne 1 ]
then
    echo "Usage: $0 {f|s|b}"
    exit 1
fi

OP=${1}

if [ ${OP} = "f" ]
then
    ros2 topic pub /tb3_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
elif [ ${OP} = "s" ]
then
    ros2 topic pub /tb3_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
elif [ ${OP} = "b" ]
then
    ros2 topic pub /tb3_cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
fi

