cmake_minimum_required(VERSION 3.8)
project(franka_sim_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch models worlds urdf config meshes rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/spawn_scene.py scripts/table_collision.py scripts/check_setup.py
  scripts/rgbd_points.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
