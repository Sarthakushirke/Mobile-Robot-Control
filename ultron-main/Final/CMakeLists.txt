cmake_minimum_required(VERSION 2.8)
project("phase 4")


find_package(OpenCV REQUIRED)

include_directories(opencv)

set(SOURCE_FILES
    src/APF.cpp
    src/tools.cpp
    src/json_restaurant.cpp
    src/objects/Grid.cpp
    src/objects/Robot.cpp)
    
    
add_library(pathPlanningMRC ${SOURCE_FILES})  
target_link_libraries(pathPlanningMRC ${OpenCV_LIBS})    

target_link_libraries(pathPlanningMRC emc-framework)
target_include_directories(pathPlanningMRC PUBLIC include/)

add_executable(main src/main.cpp)
target_link_libraries(main pathPlanningMRC)
    
    
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


