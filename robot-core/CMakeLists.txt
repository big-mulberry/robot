
cmake_minimum_required(VERSION 3.16)

project(robot_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(vector_basics
    examples/geometry/vector_basics.cpp
)

target_link_libraries(vector_basics
    PRIVATE
        Eigen3::Eigen
)

add_executable(dot_product
    examples/geometry/dot_product.cpp
)
target_link_libraries(dot_product
    PRIVATE
        Eigen3::Eigen
)

add_executable(cross_product
    examples/geometry/cross_product.cpp
)
target_link_libraries(cross_product
    PRIVATE
        Eigen3::Eigen
)

add_executable(signed_angle
    examples/geometry/signed_angle.cpp
)
target_link_libraries(signed_angle
    PRIVATE
        Eigen3::Eigen
)

add_executable(rotation_matrix.cpp
    examples/geometry/rotation_matrix.cpp
)
target_link_libraries(rotation_matrix.cpp
    PRIVATE
        Eigen3::Eigen
)

add_executable(coordinate_rotation
    examples/geometry/coordinate_rotation.cpp
)
target_link_libraries(coordinate_rotation
    PRIVATE
        Eigen3::Eigen
)

add_executable(homogeneous_transform
    examples/geometry/homogeneous_transform.cpp
)
target_link_libraries(homogeneous_transform
    PRIVATE
        Eigen3::Eigen
)

add_executable(inverse_transform
    examples/geometry/inverse_transform.cpp
)
target_link_libraries(inverse_transform
    PRIVATE
        Eigen3::Eigen
)

add_executable(transform_chain
    examples/geometry/transform_chain.cpp
)
target_link_libraries(transform_chain
    PRIVATE
        Eigen3::Eigen
)

# 轨迹规划
add_library(robot_trajectory STATIC
    src/trajectory/linear_interpolation.cpp
    src/trajectory/cubic_trajectory.cpp
    src/trajectory/quintic_trajectory.cpp
    src/trajectory/trapezoidal_profile.cpp
    src/trajectory/phase_synchronized_trajectory.cpp
)

target_include_directories(robot_trajectory
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_trajectory
    PUBLIC Eigen3::Eigen
)

add_executable(linear_interpolation_demo
    examples/trajectory/linear_interpolation_demo.cpp
)

target_link_libraries(linear_interpolation_demo
    PRIVATE robot_trajectory
)

add_executable(cubic_trajectory_demo
    examples/trajectory/cubic_trajectory_demo.cpp
)

target_link_libraries(cubic_trajectory_demo
    PRIVATE robot_trajectory
)

add_executable(quintic_trajectory_demo
    examples/trajectory/quintic_trajectory_demo.cpp
)
target_link_libraries(quintic_trajectory_demo
    PRIVATE robot_trajectory
)
add_executable(trapezoidal_plan_demo
    examples/trajectory/trapezoidal_plan_demo.cpp
)
target_link_libraries(trapezoidal_plan_demo
    PRIVATE robot_trajectory
)

add_executable(phase_sync_demo examples/trajectory/phase_sync_demo.cpp)
target_link_libraries(phase_sync_demo PRIVATE robot_trajectory)
