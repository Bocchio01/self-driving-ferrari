cmake_minimum_required(VERSION 3.20)
project(ferrari_description)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ==========================================
# Target: ackermann_to_gazebo
# ==========================================
add_executable(ackermann_to_gazebo src/ackermann_to_gazebo.cpp)
target_link_libraries(ackermann_to_gazebo
    PRIVATE
    rclcpp::rclcpp
    ackermann_msgs::ackermann_msgs
    geometry_msgs::geometry_msgs
)

# ==========================================
# Installation
# ==========================================
install(
    TARGETS
    ackermann_to_gazebo
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    urdf
    launch
    rviz
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
