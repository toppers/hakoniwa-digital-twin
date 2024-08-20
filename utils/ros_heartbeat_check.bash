#!/bin/bash

TOPIC="/TB3RoboAvatar_baggage_sensor"
TIMEOUT=10
LAST_RECEIVED=$(date +%s)

# タイムアウト監視
while true
do
  ros2 topic echo $TOPIC --once > /dev/null
  if [ $? -eq 0 ]
  then
    LAST_RECEIVED=$(date +%s)
  fi
  CURRENT_TIME=$(date +%s)
  ELAPSED=$((CURRENT_TIME - LAST_RECEIVED))

  if [ $ELAPSED -gt $TIMEOUT ]; then
    echo "Error: No data received on $TOPIC for over $TIMEOUT seconds"
  else
    echo "OK: data received on $TOPIC in $TIMEOUT seconds"
  fi

  sleep 1
done
