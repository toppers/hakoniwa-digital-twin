#!/bin/bash

source /opt/ros/humble/setup.bash

# クリーンアップ用の関数を定義
cleanup() {
    echo "Stopping nodes..."
    kill $URG_NODE_PID
}

# スクリプト終了時にクリーンアップを実行
trap cleanup EXIT

# urg_node2の起動
echo "Starting urg_node2..."
ros2 launch urg_node2 urg_node2.launch.py &
URG_NODE_PID=$!

# urg_node2が完全に起動するまで待つ (例: 5秒待機)
sleep 5

# infra_sensor_urgの起動
echo "Starting infra_sensor_urg..."
ros2 run infra_sensor_urg lidar_subscriber --ros-args -p act_mode:=real 

# 全てのプロセスが終了するのを待つ
wait $URG_NODE_PID
