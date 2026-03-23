cmake_minimum_required(VERSION 3.8)
project(fs_vehicle)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/ps_teleop
  scripts/imu_tui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  meshes
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
