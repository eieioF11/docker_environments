FROM ros:jazzy-ros-base

ENV ROS_DISTRO jazzy
ARG DEBIAN_FRONTEND=noninteractive

# tools
RUN apt-get -qq update && apt-get -q -y install \
    ca-certificates \
    build-essential \
    vim \
    net-tools \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

# ros packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-pcl-*

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
