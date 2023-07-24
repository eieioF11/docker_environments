#!/bin/bash
RUN_NAME="realsense_ros:latest"
xhost +local:user
if [ $# -eq 1 ]; then
  if [ $1 = "-g" ]; then
    echo "GPU available";
    docker run -it \
      --runtime=nvidia \
      --rm \
      --env="DISPLAY" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --env="QT_X11_NO_MITSHM=1" \
      --net host \
      --privileged \
      -v "/etc/localtime:/etc/localtime:ro" \
      $RUN_NAME \
      bash
  fi
else
  echo "GPU Disabled";
  docker run -it \
    --rm \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="QT_X11_NO_MITSHM=1" \
    --net host \
    --privileged \
    -v "/etc/localtime:/etc/localtime:ro" \
    $RUN_NAME \
    bash
fi