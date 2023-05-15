# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project("mrc-pathplanning")

set(DCMAKE_BUILD_TYPE,"Release")

# Fix flags for rangelibc and gtest
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-march=native -ffast-math -fno-math-errno")
add_definitions(-DUSE_CUDA=0)

# Include ROS libraries
find_package(catkin REQUIRED COMPONENTS roscpp)
include_directories(${catkin_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Add main executable
set(SOURCE_FILES
    src/tools.cpp
    src/Objects/Robot.cpp
    src/Planner/planning.cpp
    include/3rdparty/rangelibc/vendor/lodepng/lodepng.cpp)

add_library(pathPlanningMRC ${SOURCE_FILES})
target_link_libraries(pathPlanningMRC emc-framework)
target_link_libraries(pathPlanningMRC pthread)
target_link_libraries(pathPlanningMRC ${catkin_LIBRARIES})

target_include_directories(pathPlanningMRC PUBLIC include/)
target_include_directories(pathPlanningMRC SYSTEM INTERFACE include/3rdparty)

# # Setup gtestingc
if((NOT ${REAL_ROBOT}) OR NOT DEFINED ${REAL_ROBOT})
    include(GoogleTest)
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})
    enable_testing()

    add_executable(assignment1 tests/test_assignment1.cpp)
    target_link_libraries(assignment1 pathPlanningMRC)
    target_link_libraries(assignment1 ${GTEST_MAIN_LIBRARIES} ${GTEST_LIBRARIES})

    # # Tool Test
    add_executable(tooltest tests/tooltest.cpp)
    target_link_libraries(tooltest pathPlanningMRC)
    target_link_libraries(tooltest ${GTEST_MAIN_LIBRARIES} ${GTEST_LIBRARIES})

    gtest_discover_tests(assignment1)
    gtest_discover_tests(tooltest)
endif()
