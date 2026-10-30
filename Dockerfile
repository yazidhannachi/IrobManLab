FROM osrf/ros:jazzy-desktop AS lab
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
    ros-jazzy-moveit python3-colcon-common-extensions python3-yaml \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /home/ros2_ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /opt/franka_ws/install/setup.bash ] && source /opt/franka_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'source /home/ros2_ws/install/setup.bash' >> /root/.bashrc
ENV LAB_MODE=sim
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]

# Real FR3: libfranka and the franka_ros2 driver packages as an underlay.
# libfranka must match the robot's system version (see docs/real_robot.md).
FROM lab AS real
ARG LIBFRANKA_VERSION=0.20.5
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget ca-certificates ros-jazzy-joint-state-publisher \
    ros-jazzy-generate-parameter-library ros-jazzy-backward-ros \
    && deb=libfranka_${LIBFRANKA_VERSION}_noble_amd64.deb \
    && wget -q https://github.com/frankarobotics/libfranka/releases/download/${LIBFRANKA_VERSION}/$deb \
    && apt-get install -y --no-install-recommends ./$deb \
    && rm $deb && rm -rf /var/lib/apt/lists/*
COPY docker/franka_ros2 /opt/franka_ws/src/franka_ros2
RUN . /opt/ros/jazzy/setup.sh && cd /opt/franka_ws && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log
ENV LAB_MODE=real
