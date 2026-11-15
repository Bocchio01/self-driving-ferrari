cmake_minimum_required(VERSION 3.20)
project(ferrari_mapping)

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
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(slam_core src/occupancy_grid.cpp src/scan_matcher.cpp src/slam_core.cpp src/costmap.cpp)
target_include_directories(slam_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(slam_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS})

add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node
    PRIVATE
    slam_core
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    nav_msgs::nav_msgs
    geometry_msgs::geometry_msgs
    std_srvs::std_srvs
    tf2_ros::tf2_ros
)

install(TARGETS slam_node
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/evaluate_slam.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS slam_core
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_slam test/test_slam.cpp)
    target_link_libraries(test_slam slam_core)

    ament_add_gtest(test_costmap test/test_costmap.cpp)
    target_link_libraries(test_costmap slam_core)
endif()

ament_package()
