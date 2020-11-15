set(GTEST_SHUFFLE 1)

include_directories(include)

add_executable(
    pathPlanningTest
    main.cpp
    path_planning_test.cpp
    ../src/path_planning.cpp
)

target_include_directories(pathPlanningTest PUBLIC ../lib/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(pathPlanningTest PUBLIC gtest)
