cmake_minimum_required(VERSION 3.20)
project(ferrari_sensing)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# ==========================================
# Target: camera
# ==========================================
add_executable(camera_node src/camera.cpp)
target_include_directories(camera_node PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)
target_link_libraries(camera_node
    PRIVATE
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    ${OpenCV_LIBS}
    cv_bridge::cv_bridge
)

# ==========================================
# Installation
# ==========================================
install(TARGETS
    camera_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY
    launch
    config
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
