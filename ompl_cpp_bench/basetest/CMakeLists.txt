cmake_minimum_required(VERSION 3.16)
project(ompl_se3_benchmark CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ompl REQUIRED)
add_executable(se3_room_benchmark se3_room_benchmark.cpp)
target_link_libraries(se3_room_benchmark PRIVATE ompl::ompl)