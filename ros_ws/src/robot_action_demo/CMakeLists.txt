cmake_minimum_required(VERSION 3.8)
project(robot_action_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

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

add_executable(
    fibonacci_action_server
    src/fibonacci_action_server.cpp
    src/fibonacci_action_server_main.cpp
)

target_include_directories(
    fibonacci_action_server
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(
    fibonacci_action_server
    PRIVATE
        cxx_std_17
)

ament_target_dependencies(
    fibonacci_action_server
    rclcpp
    rclcpp_action
    example_interfaces
)

install(
    TARGETS fibonacci_action_server
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(fibonacci_action_client
    src/fibonacci_action_client.cpp
    src/fibonacci_action_client_main.cpp
)

target_include_directories(fibonacci_action_client
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(fibonacci_action_client
    PRIVATE cxx_std_17
)

ament_target_dependencies(fibonacci_action_client
    rclcpp
    rclcpp_action
    example_interfaces
)

install(
    TARGETS fibonacci_action_client
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
