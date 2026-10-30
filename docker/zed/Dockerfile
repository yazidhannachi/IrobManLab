# ZED X Mini receiver for the real robot: the ZED SDK and its ROS 2 wrapper turn
# the Jetson's network stream into the lab's /zed_x_mini/* topics. Built and run
# with ./scripts/build.sh --zed and ./scripts/run.sh --real --zed (NVIDIA GPU).
# The receiver's ZED SDK should match the sender's major.minor version.
ARG ZED_IMAGE=stereolabs/zed:5.4-gl-devel-cuda12.8-ubuntu24.04
FROM ${ZED_IMAGE}
ARG ZED_WRAPPER_VERSION=v5.4.1
ENV DEBIAN_FRONTEND=noninteractive

# ROS 2 Jazzy, as in the wrapper's own Docker build.
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl ca-certificates gnupg2 lsb-release software-properties-common git \
    && add-apt-repository -y universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base python3-rosdep python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/zed_ws
RUN git clone --depth 1 --branch ${ZED_WRAPPER_VERSION} \
      https://github.com/stereolabs/zed-ros2-wrapper.git src/zed-ros2-wrapper
# The desktop base image has no OpenCV, so rosdep installs Ubuntu's with the
# other ROS dependencies (the wrapper's own build holds it only for Jetson).
RUN apt-get update && (rosdep init || true) && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-up-to zed_wrapper --cmake-args -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS=-Wl,--allow-shlib-undefined \
    && rm -rf build log

COPY docker/zed/zed_stream.launch.py docker/zed/zed_stream.yaml /opt/zed_lab/
CMD ["bash", "-c", ". /opt/zed_ws/install/setup.bash && exec ros2 launch /opt/zed_lab/zed_stream.launch.py"]
