cmake_minimum_required(VERSION 3.11.3)

add_definitions(-std=c++17)

set(CXX_FLAGS, "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

include_directories(third_party/Eigen)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

project(simulator)

file(GLOB project_SRCS src/*.cpp)

add_executable(simulator ${project_SRCS})
target_link_libraries (simulator Eigen3::Eigen)
