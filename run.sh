#!/bin/sh

xhost +local:user
docker run -it \
    --privileged \
    --runtime=nvidia \
    --rm \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="QT_X11_NO_MITSHM=1" \
    --name azure_kinect \
    --net host \
    --privileged \
    --env CLIENT_IP=133.15.23.61 \
    --env MASTER_IP=133.15.23.61 \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/$(pwd)/ros_packages/Azure_Kinect_ROS_Driver:/home/catkin_ws/src/Azure_Kinect_ROS_Driver" \
    azure_kinect:latest
