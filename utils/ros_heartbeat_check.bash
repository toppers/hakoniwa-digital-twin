#!/bin/bash

TOPIC="/TB3RoboAvatar_baggage_sensor"
TIMEOUT=10
LAST_RECEIVED=$(date +%s)

# トピックをサブスクライブし、メッセージを受信するたびに時間を更新する
ros2 topic echo $TOPIC --once | while read -r line
do
  LAST_RECEIVED=$(date +%s)
done &

# タイムアウト監視
while true
do
  CURRENT_TIME=$(date +%s)
  ELAPSED=$((CURRENT_TIME - LAST_RECEIVED))

  if [ $ELAPSED -gt $TIMEOUT ]; then
    echo "Error: No data received on $TOPIC for over $TIMEOUT seconds"
  else
    echo "OK: data received on $TOPIC in $TIMEOUT seconds"
  fi

  sleep 3
done
