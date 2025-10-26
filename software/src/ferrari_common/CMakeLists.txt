cmake_minimum_required(VERSION 3.14)

# Set the project name
project(ferrari_common)

# Require C++17 for ROS 2 interface generation
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # Dependency for your messages

# Find rosidl_default_generators for message/service generation
find_package(rosidl_default_generators REQUIRED)

# List message and service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/motion_cmd.msg"
  "msg/control_cmd.msg"
  "srv/get_gate_mode.srv"
  "srv/set_gate_mode.srv"
  "srv/toggle_arm_vehicle.srv"
  "srv/get_kinematic_model.srv"
  DEPENDENCIES std_msgs
)

# Export interfaces for downstream packages
ament_export_dependencies(std_msgs)
ament_export_interfaces(${PROJECT_NAME})

# Install targets (optional, but good practice)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
