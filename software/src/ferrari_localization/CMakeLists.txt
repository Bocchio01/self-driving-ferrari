cmake_minimum_required(VERSION 3.20)
project(ferrari_localization)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)

# ==========================================
# Installation
# ==========================================
install(PROGRAMS
    scripts/odom_qos_bridge.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

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
