cmake_minimum_required(VERSION 3.8)
project(custom_drive_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(drive_publisher src/drive_publisher.cpp)

ament_target_dependencies(drive_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  drive_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()