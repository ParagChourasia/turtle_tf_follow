cmake_minimum_required(VERSION 3.8)
project(turtle_tf_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Find dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ------------------------------------------------------------------------------
# Add include directories (Fix for turtlesim headers)
# ------------------------------------------------------------------------------
include_directories(
  include
  /opt/ros/humble/include/turtlesim     # ★ IMPORTANT FIX ★
)

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------

add_executable(leader_tf_broadcaster src/leader_tf_broadcaster.cpp)
ament_target_dependencies(leader_tf_broadcaster
  rclcpp
  geometry_msgs
  tf2_ros
  turtlesim
)

add_executable(turtle2_tf_broadcaster src/turtle2_tf_broadcaster.cpp)
ament_target_dependencies(turtle2_tf_broadcaster
  rclcpp
  geometry_msgs
  tf2_ros
  turtlesim
)

add_executable(follower_controller src/follower_controller.cpp)
ament_target_dependencies(follower_controller
  rclcpp
  geometry_msgs
  tf2_ros
  turtlesim
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Installation
# ------------------------------------------------------------------------------
install(TARGETS
  leader_tf_broadcaster
  turtle2_tf_broadcaster
  follower_controller
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
