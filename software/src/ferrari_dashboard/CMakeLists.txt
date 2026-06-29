cmake_minimum_required(VERSION 3.20)
project(ferrari_dashboard)

find_package(ament_cmake REQUIRED)

# ==========================================
# Installation
# ==========================================
install(PROGRAMS
    src/bridge.js
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    public/
    DESTINATION share/${PROJECT_NAME}/public
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


