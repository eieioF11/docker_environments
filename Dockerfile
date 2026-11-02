FROM ros:jazzy-ros-core

ENV ROS_DISTRO jazzy
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get -qq update && apt-get -q -y --no-install-recommends install \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-perception-pcl \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
