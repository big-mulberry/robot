cmake_minimum_required(VERSION 3.16)

project(robot_hello LANGUAGES CXX)

add_executable(hello_world main.cpp)
target_compile_features(hello_world PRIVATE cxx_std_17)
set_target_properties(hello_world PROPERTIES CXX_EXTENSIONS OFF)
