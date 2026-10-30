cmake_minimum_required(VERSION 3.8)
project(franka_real_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch urdf config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/fr3_gripper_bridge.py scripts/workspace_collision.py
  scripts/check_real_setup.py scripts/arm_motion_test.py scripts/robot_settings.py
  scripts/trajectory_guard.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
