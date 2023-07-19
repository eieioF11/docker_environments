# FROM nvidia/cudagl:11.0-base
FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get -qq update && apt-get -q -y install tzdata \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

# ENV DEBIAN_FRONTEND noninteractive

RUN apt-get -qq update && apt-get -q -y install \
    gnupg2 \
    curl \
    ca-certificates \
    build-essential \
    git \
    tmux \
    nano \
    wget \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

# setup sources.list and keys
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list \
    && curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
# install ROS (including dependencies for building packages) and other packages
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get -qq update && apt-get -q -y install \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-noetic-desktop \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

# sdformat8-sdf conflicts with sdformat-sdf installed from gazebo
# so we need to workaround this using a force overwrite
# Do this before installing ign-gazebo
# (then install ign-blueprint and ros to ign bridge)
RUN echo "deb [trusted=yes] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get -qq update && apt-get -q -y install \
    ignition-dome \
    ros-noetic-ros-ign \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean


#COPY . /catkin_ws/src
#WORKDIR /catkin_ws

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    net-tools \
    blender \
    ros-noetic-pcl-ros \
    ros-noetic-geometry2 \
    ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-gmapping

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# RUN rosdep update
# RUN rosdep install --from-paths src --ignore-src -r -y  || echo "There were some errors during rosdep install"
# SHELL ["/bin/bash", "-c"]
# RUN source /opt/ros/noetic/setup.bash && \
#     catkin_make
