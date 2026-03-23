cmake_minimum_required(VERSION 3.8)
project(fs_vehicle_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)

# ── Gazebo vendor prefix paths ──────────────────────────────────────────────
# These are needed so CMake can locate the gz-sim8, gz-transport13, etc.
# config files installed by the ROS2 jazzy vendor packages.
list(APPEND CMAKE_PREFIX_PATH
  "/opt/ros/jazzy/opt/gz_sim_vendor"
  "/opt/ros/jazzy/opt/gz_transport_vendor"
  "/opt/ros/jazzy/opt/gz_msgs_vendor"
  "/opt/ros/jazzy/opt/gz_plugin_vendor"
  "/opt/ros/jazzy/opt/gz_math_vendor"
  "/opt/ros/jazzy/opt/gz_common_vendor"
  "/opt/ros/jazzy/opt/gz_utils_vendor"
  "/opt/ros/jazzy/opt/sdformat_vendor"
  "/opt/ros/jazzy/opt/gz_cmake_vendor"
)

find_package(gz-sim8   REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10  REQUIRED)
find_package(gz-math7   REQUIRED)

# ── FsAckermannDrive plugin ─────────────────────────────────────────────────
add_library(FsAckermannDrive SHARED
  src/fs_ackermann_drive.cpp
)

target_include_directories(FsAckermannDrive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(FsAckermannDrive
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
)

# Install the .so so Gazebo can find it
install(TARGETS FsAckermannDrive
  LIBRARY DESTINATION lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
