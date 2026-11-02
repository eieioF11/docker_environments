FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/ros2_ws

# 1. 必要なシステム依存パッケージとPPA追加用パッケージ（software-properties-common）のインストール
RUN apt-get update && apt-get install -y \
    software-properties-common \
    git \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    libboost-all-dev \
    libpcl-dev \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

# GTSAMライブラリのインストール
RUN apt-get update && apt-get install -y libgtsam-dev

# 2. PPAからIridescenceを追加してインストール
RUN add-apt-repository -y ppa:koide3/iridescence && \
    apt-get update && apt-get install -y \
    libiridescence-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. 対象パッケージのクローンとサブモジュールの初期化
WORKDIR $ROS_WS/src
RUN git clone -b devel https://github.com/eieioF11/simple_slam.git && \
    git clone https://github.com/eieioF11/ros2_common_tools.git && \
    cd ros2_common_tools && \
    git submodule update --init --recursive

WORKDIR $ROS_WS

# 4. rosdepによるROS依存関係の解決
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN touch /usr/lib/aarch64-linux-gnu/libCppUnitLite.a

# 5. ワークスペースのビルド
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# コンテナ起動時に環境変数を自動ロード
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
# COPY ./ros_entrypoint.sh /ros_entrypoint.sh
# RUN chmod +x /ros_entrypoint.sh
# ENTRYPOINT ["/ros_entrypoint.sh"]
