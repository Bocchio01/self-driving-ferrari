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
# Shared teleop_base
# ==========================================
add_library(teleop_base src/teleop/teleop_base.cpp)
target_include_directories(teleop_base PUBLIC include)
target_link_libraries(teleop_base
    PUBLIC
    rclcpp::rclcpp
    ackermann_msgs::ackermann_msgs
    std_srvs::std_srvs
)

# ==========================================
# Target 1: teleop_joy_node
# ==========================================
add_executable(teleop_joy_node src/teleop/teleop_joy.cpp)
target_include_directories(teleop_joy_node PUBLIC include)
target_link_libraries(teleop_joy_node
    PRIVATE
    teleop_base
    sensor_msgs::sensor_msgs
)

# ==========================================
# Target 2: teleop_keyboard_node
# ==========================================
add_executable(teleop_keyboard_node src/teleop/teleop_keyboard.cpp src/teleop/KBHit.cpp)
target_include_directories(teleop_keyboard_node PUBLIC include)
target_link_libraries(teleop_keyboard_node
    PRIVATE
    teleop_base
)

# ==========================================
# Target 3: control_gate_node
# ==========================================
add_executable(control_gate_node src/control_gate.cpp)
target_include_directories(control_gate_node PUBLIC include)
target_link_libraries(control_gate_node
    PRIVATE
    rclcpp::rclcpp
    ackermann_msgs::ackermann_msgs
    std_srvs::std_srvs
)

# ==========================================
# Installation
# ==========================================
install(TARGETS
    teleop_base
    teleop_joy_node
    teleop_keyboard_node
    control_gate_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
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
