cmake_minimum_required(VERSION 3.8)
project(robot_status_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(robot_controller
    src/robot_controller.cpp
)

target_compile_features(robot_controller
    PRIVATE cxx_std_17
)

ament_target_dependencies(robot_controller
    rclcpp
    std_msgs
)

install(
    TARGETS robot_controller
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(robot_monitor
    src/robot_monitor.cpp
)

target_compile_features(robot_monitor
    PRIVATE cxx_std_17
)

ament_target_dependencies(robot_monitor
    rclcpp
    std_msgs
)

install(
    TARGETS robot_monitor
    DESTINATION lib/${PROJECT_NAME}
)
### service server
add_executable(status_service_server
    src/status_service_server.cpp
    src/status_service_server_main.cpp
)

target_include_directories(status_service_server
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(status_service_server
    PRIVATE cxx_std_17
)

ament_target_dependencies(status_service_server
    rclcpp
    std_srvs
)
### service client
add_executable(status_service_client
    src/status_service_client.cpp
    src/status_service_client_main.cpp
)

target_include_directories(status_service_client
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(status_service_client
    PRIVATE cxx_std_17
)

ament_target_dependencies(status_service_client
    rclcpp
    std_srvs
)


install(
    TARGETS
        status_service_server
        status_service_client
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
