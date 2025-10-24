cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

# Warnings for C++
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies (build system) ---
find_package(ament_cmake REQUIRED)
# Gazebo / Ignition interfaces
find_package(ros_ign_interfaces REQUIRED)

# --- Install data (unchanged) ---
install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_tree_to_map.py
  scripts/odom_to_tf.py
  scripts/topic_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Tests (unchanged) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()