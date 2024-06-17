cmake_minimum_required(VERSION 2.8)
project(my_project)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(
    include
)

add_executable(example src/main.cpp src/sample.cpp)

# Link executable to the emc-enviroment
target_link_libraries(example emc-framework)

# Link executable to OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(example ${OpenCV_LIBS})
