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
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(ferrari_vehicle REQUIRED)
find_package(ferrari_planning REQUIRED)

# ==========================================
# Shared trajectory_utils
# ==========================================
add_library(trajectory_utils src/trajectory_utils.cpp)
target_include_directories(trajectory_utils PUBLIC include)
target_link_libraries(trajectory_utils
    PUBLIC
    Eigen3::Eigen
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    geometry_msgs::geometry_msgs
    ferrari_planning::ferrari_planning
    ferrari_vehicle::vehicle_models
)

# ==========================================
# Shared controller_interface
# ==========================================
add_library(controller_interface src/controller.cpp)
target_include_directories(controller_interface PUBLIC include)
target_link_libraries(controller_interface
    PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    nav_msgs::nav_msgs
    ackermann_msgs::ackermann_msgs
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ferrari_planning::ferrari_planning
    ferrari_vehicle::vehicle_models
)

# =========================================
# Shared mpc_solver
# =========================================
add_library(mpc_solver src/solvers/mpc_solver.cpp)
target_include_directories(mpc_solver PUBLIC include)
target_link_libraries(mpc_solver
    PUBLIC
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
    ferrari_vehicle::vehicle_models
)

# ==========================================
# Target joypad_controller_node
# ==========================================
add_executable(joypad_controller_node src/controllers/joypad_controller.cpp)
target_include_directories(joypad_controller_node PUBLIC include)
target_link_libraries(joypad_controller_node
    PRIVATE
    # trajectory_utils # Not needed for joypad controller
    controller_interface
    rclcpp::rclcpp
    std_srvs::std_srvs
    sensor_msgs::sensor_msgs
)

# ==========================================
# Target lateral_mpc_controller_node
# ==========================================
add_executable(lateral_mpc_controller_node src/controllers/lateral_mpc_controller.cpp)
target_include_directories(lateral_mpc_controller_node PUBLIC include)
target_link_libraries(lateral_mpc_controller_node
    PRIVATE
    trajectory_utils
    controller_interface
    mpc_solver
    rclcpp::rclcpp
    ferrari_vehicle::vehicle_models
)

# ==========================================
# Target lateral_pure_pursuit_controller_node
# ==========================================
add_executable(lateral_pure_pursuit_controller_node src/controllers/lateral_pure_pursuit_controller.cpp)
target_include_directories(lateral_pure_pursuit_controller_node PUBLIC include)
target_link_libraries(lateral_pure_pursuit_controller_node
    PRIVATE
    trajectory_utils
    controller_interface
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ferrari_vehicle::vehicle_models
)

# ==========================================
# Target lateral_stanley_controller_node
# ==========================================
add_executable(lateral_stanley_controller_node src/controllers/lateral_stanley_controller.cpp)
target_include_directories(lateral_stanley_controller_node PUBLIC include)
target_link_libraries(lateral_stanley_controller_node
    PRIVATE
    trajectory_utils
    controller_interface
    rclcpp::rclcpp
    ferrari_vehicle::vehicle_models
)

# ==========================================
# Target longitudinal_pid_controller_node
# ==========================================
add_executable(longitudinal_pid_controller_node src/controllers/longitudinal_pid_controller.cpp)
target_include_directories(longitudinal_pid_controller_node PUBLIC include)
target_link_libraries(longitudinal_pid_controller_node
    PRIVATE
    trajectory_utils
    controller_interface
    rclcpp::rclcpp
)

# ==========================================
# Target mpc_controller_node
# ==========================================
add_executable(mpc_controller_node src/controllers/mpc_controller.cpp)
target_include_directories(mpc_controller_node PUBLIC include)
target_link_libraries(mpc_controller_node
    PRIVATE
    trajectory_utils
    controller_interface
    mpc_solver
    rclcpp::rclcpp
    ferrari_vehicle::vehicle_models
)

# ==========================================
# Target controller_multiplexer_node
# ==========================================
add_executable(controller_multiplexer_node src/controller_multiplexer.cpp)
target_include_directories(controller_multiplexer_node PUBLIC include)
target_link_libraries(controller_multiplexer_node
    PRIVATE
    rclcpp::rclcpp
    lifecycle_msgs::lifecycle_msgs
    ackermann_msgs::ackermann_msgs
    std_srvs::std_srvs
)

# ==========================================
# Installation
# ==========================================
install(TARGETS
    joypad_controller_node
    lateral_mpc_controller_node
    lateral_pure_pursuit_controller_node
    lateral_stanley_controller_node
    longitudinal_pid_controller_node
    mpc_controller_node
    controller_multiplexer_node
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/controller_visualizer.py
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
