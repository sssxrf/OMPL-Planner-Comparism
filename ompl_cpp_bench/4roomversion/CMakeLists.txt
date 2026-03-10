# CMakeLists.txt (in your 4roomversion folder)
cmake_minimum_required(VERSION 3.16)
project(ompl_se3_tour_one_run CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ompl REQUIRED)

add_executable(se3_room_tour_one_run se3_room_tour_one_run.cpp)
target_link_libraries(se3_room_tour_one_run PRIVATE ompl::ompl)