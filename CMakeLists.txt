cmake_minimum_required(VERSION 3.10.2)
project(path_planning)

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11")

find_package( Boost REQUIRED COMPONENTS system)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

add_executable (
  pathPlanning
  src/main.cpp 
  src/path_planning.cpp)

add_subdirectory(lib/googletest)
add_subdirectory(test)
