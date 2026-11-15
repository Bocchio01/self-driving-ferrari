cmake_minimum_required(VERSION 3.20)
project(ferrari_perception)

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
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/LineState.msg"
    DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(line_detection src/line_detection.cpp)
target_include_directories(line_detection PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(line_detection PUBLIC Eigen3::Eigen ${OpenCV_LIBS})

# ==========================================
# Target: line_detector
# ==========================================
add_executable(line_detector_node src/line_detector.cpp)
target_include_directories(line_detector_node PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(line_detector_node
    PRIVATE
    line_detection
    ${cpp_typesupport_target}
    geometry_msgs::geometry_msgs
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    ${OpenCV_LIBS}
    cv_bridge::cv_bridge
    tf2::tf2
    image_transport::image_transport
    nav_msgs::nav_msgs
)

# ==========================================
# Installation
# ==========================================
install(TARGETS
    line_detector_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS line_detection
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
    scripts
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
    ament_add_gtest(test_line_detection test/test_line_detection.cpp)
    target_link_libraries(test_line_detection line_detection)
    target_compile_definitions(test_line_detection PRIVATE TEST_IMAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scripts/images")
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
