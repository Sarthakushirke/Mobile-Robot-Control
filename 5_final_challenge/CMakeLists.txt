# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project("mrc-pathplanning")

set(DCMAKE_BUILD_TYPE,"Release")

# Fix flags for rangelibc and gtest
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-march=native -ffast-math -fno-math-errno")
add_definitions(-DUSE_CUDA=0)

# Include libraries
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Add main executable
set(SOURCE_FILES
    src/tools.cpp
    src/Objects/Robot.cpp
    src/Planner/planning.cpp
    src/PRM/prm.cpp
    src/Planner/Pos.cpp
    src/Planner/VFH.cpp
    # src/update_map.cpp
    include/3rdparty/rangelibc/vendor/lodepng/lodepng.cpp)

add_library(pathPlanningMRC ${SOURCE_FILES})
target_link_libraries(pathPlanningMRC emc-framework)
target_link_libraries(pathPlanningMRC pthread)
target_link_libraries(pathPlanningMRC ${catkin_LIBRARIES})

target_include_directories(pathPlanningMRC PUBLIC include/)
target_include_directories(pathPlanningMRC SYSTEM INTERFACE include/3rdparty)

add_executable(follow_astar src/main.cpp)
target_link_libraries(follow_astar emc-framework pathPlanningMRC ${OpenCV_LIBS})

# add_executable(local_nav main.cpp)
# target_link_libraries(follow_astar emc-framework pathPlanningMRC ${OpenCV_LIBS})

# # Setup gtestingc
if((NOT ${REAL_ROBOT}) OR NOT DEFINED ${REAL_ROBOT})
    include(GoogleTest)
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})
    enable_testing()

    # add_executable(assignment tests/test_assignment.cpp)
    # target_link_libraries(assignment pathPlanningMRC)
    # target_link_libraries(assignment ${GTEST_MAIN_LIBRARIES} ${GTEST_LIBRARIES} ${OpenCV_LIBS})

    # # # Tool Test
    # add_executable(tooltest tests/tooltest.cpp)
    # target_link_libraries(tooltest pathPlanningMRC)
    # target_link_libraries(tooltest ${GTEST_MAIN_LIBRARIES} ${GTEST_LIBRARIES} ${OpenCV_LIBS})
endif()

# cmake_minimum_required(VERSION 2.8)
# project(local_nav)

# add_executable(local_nav ../2_local_navigation/src/Local_Nav.cpp ../2_local_navigation/src/Pos.cpp ../2_local_navigation/src/VFH.cpp) 
# target_link_libraries(local_nav emc-framework)
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
