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

add_library(exposure_controller src/exposure_controller.cpp)
target_include_directories(exposure_controller PUBLIC include)

add_library(ld19_parser src/ld19_parser.cpp)
target_include_directories(ld19_parser PUBLIC include)

add_executable(lidar_node src/lidar_node.cpp)
target_link_libraries(lidar_node
    PRIVATE
    ld19_parser
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
)

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
    lidar_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS exposure_controller ld19_parser
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
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

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_exposure_controller test/test_exposure_controller.cpp)
    target_link_libraries(test_exposure_controller exposure_controller)

    ament_add_gtest(test_ld19_parser test/test_ld19_parser.cpp)
    target_link_libraries(test_ld19_parser ld19_parser)
endif()

ament_package()
