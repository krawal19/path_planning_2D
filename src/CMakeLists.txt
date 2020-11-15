add_executable (
  pathPlanning
  main.cpp 
  path_planning.cpp)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)
