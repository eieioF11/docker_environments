FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/ros2_ws
ARG BUILD_JOBS=1

# 1. 必要なシステム依存パッケージ、PPA追加用パッケージ、GTSAMのインストール
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
    libgtsam-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. PPAからIridescenceを追加してインストール
RUN add-apt-repository -y ppa:koide3/iridescence && \
    apt-get update && apt-get install -y \
    libiridescence-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR $ROS_WS/src

# 3. 対象パッケージのクローン（サブモジュールを再帰的に取得）
RUN git clone -b devel --recursive https://github.com/eieioF11/simple_slam.git && \
    git clone --recursive https://github.com/eieioF11/ros2_common_tools.git

WORKDIR $ROS_WS

# 4. rosdepによるROS依存関係の解決
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# 5. GTSAMのパッケージングバグ回避用ダミーファイルを作成
RUN if [ -d /usr/lib/aarch64-linux-gnu ]; then touch /usr/lib/aarch64-linux-gnu/libCppUnitLite.a; fi && \
    if [ -d /usr/lib/x86_64-linux-gnu ]; then touch /usr/lib/x86_64-linux-gnu/libCppUnitLite.a; fi

# 6. ワークスペースのビルド (スレッド数制限)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} && \
    colcon build --symlink-install --parallel-workers ${BUILD_JOBS} --cmake-args -DCMAKE_BUILD_TYPE=Release"

# コンテナ起動時に環境変数を自動ロード
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

CMD ["bash"]