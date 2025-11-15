cmake_minimum_required(VERSION 3.8)
project(custom_drive_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DriveCommand.srv"
  DEPENDENCIES geometry_msgs
)

# Add executables
add_executable(drive_publisher src/drive_publisher.cpp)
add_executable(drive_service src/drive_service.cpp)

# Set dependencies for drive_publisher
ament_target_dependencies(drive_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Set dependencies for drive_service
ament_target_dependencies(drive_service
  rclcpp
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link custom service to drive_service
rosidl_target_interfaces(drive_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install both executables
install(TARGETS
  drive_publisher
  drive_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

