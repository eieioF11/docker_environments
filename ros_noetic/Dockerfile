FROM osrf/ros:noetic-desktop-full

ENV ROS_DISTRO noetic
ARG DEBIAN_FRONTEND=noninteractive

# tools
RUN apt-get -qq update && apt-get -q -y install \
    gnupg2 \
    curl \
    ca-certificates \
    build-essential \
    git \
    tmux \
    nano \
    wget \
    net-tools \
    blender \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

# ros packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-geometry2 \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-gmapping \
    ros-${ROS_DISTRO}-scan-tools \
    ros-${ROS_DISTRO}-rgbd-launch \
    ros-${ROS_DISTRO}-joy

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
