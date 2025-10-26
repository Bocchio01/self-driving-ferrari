cmake_minimum_required(VERSION 3.14)  # ROS 2 recommends at least CMake 3.14
project(ferrari_human_machine_interface)

# Require C++17 for ROS 2
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ferrari_common REQUIRED)  # Your interface package

# Declare executables
add_executable(joy_interface src/joy_interface.cpp src/control_interface.cpp)
add_executable(keyboard_interface src/keyboard_interface.cpp src/control_interface.cpp src/KBHit.cpp)

# Add dependencies for executables
ament_target_dependencies(joy_interface rclcpp std_msgs ferrari_common)
ament_target_dependencies(keyboard_interface rclcpp std_msgs ferrari_common)

# Link libraries (No need to manually link rclcpp here as ament_target_dependencies handles this)
# target_link_libraries(joy_interface rclcpp::rclcpp)  # Not needed in ROS 2
# target_link_libraries(keyboard_interface rclcpp::rclcpp)  # Not needed in ROS 2

# Install executables
install(TARGETS joy_interface keyboard_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()  # Finalize the package setup
