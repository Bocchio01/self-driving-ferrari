cmake_minimum_required(VERSION 3.20)
project(ferrari_localization)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(state_estimator src/state_estimator.cpp)
target_include_directories(state_estimator PUBLIC include)
target_link_libraries(state_estimator PUBLIC Eigen3::Eigen)

add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node
    PRIVATE
    state_estimator
    rclcpp::rclcpp
    nav_msgs::nav_msgs
    sensor_msgs::sensor_msgs
    geometry_msgs::geometry_msgs
    tf2_ros::tf2_ros
)

# ==========================================
# Installation
# ==========================================
install(TARGETS
    localization_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    config
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

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_state_estimator test/test_state_estimator.cpp)
    target_link_libraries(test_state_estimator state_estimator)
endif()

ament_package()
