cmake_minimum_required(VERSION 3.14)  # Recommended minimum for ROS 2
project(ferrari_control)

# Set C++ standard to C++17 (common in ROS 2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ferrari_common REQUIRED)  # Interface package for messages/services

# Include headers (no need to manually add directories if using ament)
# include_directories(include)  # This is handled by ament

# Add executable target
add_executable(vehicle_cmd_gate src/vehicle_cmd_gate.cpp)

# Link libraries (no need to manually link rclcpp; it's handled by ament)
# target_link_libraries(vehicle_cmd_gate rclcpp::rclcpp)  # Not needed with ament_target_dependencies

# Make sure the executable depends on necessary packages
ament_target_dependencies(vehicle_cmd_gate
  rclcpp
  std_msgs
  ferrari_common  # Ensure we link to ferrari_common interfaces
)

# Install the executable (common practice for ROS 2 packages)
install(TARGETS vehicle_cmd_gate
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()  # Finalizes the package setup
