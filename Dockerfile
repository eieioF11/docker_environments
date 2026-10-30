FROM tiryoh/ros2-desktop-vnc:jazzy

ENV ROS_DISTRO jazzy
ARG DEBIAN_FRONTEND=noninteractive

# tools
RUN apt-get -qq update && apt-get -q -y install \
    gnupg2 \
    curl \
    ca-certificates \
    build-essential \
    git \
    gh \
    tmux \
    vim \
    wget \
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
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-pcl-* \
    ros-${ROS_DISTRO}-geometry2 \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joy

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
