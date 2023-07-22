#!/bin/sh

xhost +local:user
docker run -it \
  --rm \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="QT_X11_NO_MITSHM=1" \
  --net host \
  --privileged \
  -v "/etc/localtime:/etc/localtime:ro" \
  realsense_ros:latest \
  bash