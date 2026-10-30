#!/bin/bash
set -e
rm -f /tmp/lab-ready
source /opt/ros/jazzy/setup.bash
# The real-robot image adds libfranka and the franka_ros2 driver as an underlay.
if [ -f /opt/franka_ws/install/setup.bash ]; then
  source /opt/franka_ws/install/setup.bash
fi
cd /home/ros2_ws
# Separate named volumes start empty; no instructor binaries are reused.
colcon build --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
touch /tmp/lab-ready
exec "$@"
