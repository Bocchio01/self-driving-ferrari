cmake_minimum_required(VERSION 3.20)
project(ferrari_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ==========================================
# Message generation
# ==========================================
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/OdomLite.msg"
    DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ==========================================
# Shared vehicle_models
# ==========================================
add_library(vehicle_models INTERFACE)
target_include_directories(vehicle_models INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(vehicle_models
    INTERFACE
    Eigen3::Eigen
)


# ==========================================
# Target: odom_proxy_node
# ==========================================
add_executable(odom_proxy_node src/odom_proxy.cpp)
target_include_directories(odom_proxy_node PUBLIC include)
target_link_libraries(odom_proxy_node
    PRIVATE
    rclcpp::rclcpp
    nav_msgs::nav_msgs
    ${cpp_typesupport_target}
)

# ==========================================
# Target: ferrari_twin_node
# ==========================================
add_executable(ferrari_twin_node src/ferrari_twin.cpp)
target_include_directories(ferrari_twin_node PUBLIC include)
target_link_libraries(ferrari_twin_node
    PRIVATE
    vehicle_models
    rclcpp::rclcpp
    nav_msgs::nav_msgs
    ackermann_msgs::ackermann_msgs
    geometry_msgs::geometry_msgs
    tf2::tf2
    tf2_ros::tf2_ros
    ${cpp_typesupport_target}
)

# ==========================================
# Installation
# ==========================================

install(
    TARGETS vehicle_models
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

install(TARGETS
    odom_proxy_node
    ferrari_twin_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
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
