#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <local_directory>"
  exit 1
fi

LOCAL_DIR=$(realpath $1)

docker run -it --rm \
  --net=host \
  --env="DISPLAY" \
  --volume="$LOCAL_DIR:/home/ros/workspace" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --workdir="/home/ros/workspace" \
  osrf/ros:foxy-desktop

