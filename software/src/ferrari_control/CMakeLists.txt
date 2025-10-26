cmake_minimum_required(VERSION 3.20)
project(ferrari_control)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ==========================================
# Shared teleop base
# ==========================================
add_library(ferrari_control_teleop_base src/teleop_base.cpp)
target_include_directories(ferrari_control_teleop_base PUBLIC include)
target_link_libraries(ferrari_control_teleop_base
    PUBLIC
    rclcpp::rclcpp
    ackermann_msgs::ackermann_msgs
    std_srvs::std_srvs
)

# ==========================================
# Target 1: joy_teleop_node
# ==========================================
add_executable(joy_teleop_node src/joy_teleop.cpp)
target_include_directories(joy_teleop_node PUBLIC include)
target_link_libraries(joy_teleop_node
    PRIVATE
    ferrari_control_teleop_base
    sensor_msgs::sensor_msgs
)

# ==========================================
# Target 2: keyboard_teleop_node
# ==========================================
add_executable(keyboard_teleop_node src/keyboard_teleop.cpp src/KBHit.cpp)
target_include_directories(keyboard_teleop_node PUBLIC include)
target_link_libraries(keyboard_teleop_node
    PRIVATE
    ferrari_control_teleop_base
)

# ==========================================
# Target 3: vehicle_cmd_gate_node
# ==========================================
add_executable(vehicle_cmd_gate_node src/vehicle_cmd_gate.cpp)
target_include_directories(vehicle_cmd_gate_node PUBLIC include)
target_link_libraries(vehicle_cmd_gate_node
    PRIVATE
    rclcpp::rclcpp
    ackermann_msgs::ackermann_msgs
    std_srvs::std_srvs
)

# ==========================================
# Installation
# ==========================================
install(TARGETS
    ferrari_control_teleop_base
    joy_teleop_node
    keyboard_teleop_node
    vehicle_cmd_gate_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

# =========================================
# Testing
# =========================================
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
