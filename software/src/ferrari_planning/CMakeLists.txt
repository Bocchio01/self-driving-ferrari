cmake_minimum_required(VERSION 3.20)
project(ferrari_planning)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ==========================================
# Message generation
# ==========================================
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/TrajectoryPoint.msg"
    "msg/Trajectory.msg"
    DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ==========================================
# Target global_planner_node
# ==========================================
# add_executable(global_planner_node src/global_planner.cpp)
# target_include_directories(global_planner_node PUBLIC include)
# target_link_libraries(global_planner_node
#     PRIVATE
#     rclcpp::rclcpp
#     nav_msgs::nav_msgs
#     tf2::tf2
#     tf2_ros::tf2_ros
#     tf2_geometry_msgs::tf2_geometry_msgs
# )

# ==========================================
# Target local_planner_node
# ==========================================
add_executable(local_planner_node src/local_planner.cpp)
target_include_directories(local_planner_node PUBLIC include)
target_link_libraries(local_planner_node
    PRIVATE
    rclcpp::rclcpp
    nav_msgs::nav_msgs
    sensor_msgs::sensor_msgs
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
)

# ==========================================
# Target trajectory_generator_node
# ==========================================
add_executable(trajectory_generator_node src/trajectory_generator.cpp)
target_include_directories(trajectory_generator_node PUBLIC include)
target_link_libraries(trajectory_generator_node
    PRIVATE
    rclcpp::rclcpp
    nav_msgs::nav_msgs
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${cpp_typesupport_target}
)

# ==========================================
# Installation
# ==========================================
install(TARGETS
    # global_planner_node
    local_planner_node
    trajectory_generator_node
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/mock_global_planner.py
    scripts/visualizer.py
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
