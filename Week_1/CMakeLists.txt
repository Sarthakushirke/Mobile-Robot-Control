cmake_minimum_required(VERSION 2.8)
project(Week_1)
add_executable(hello src/hello.cpp)
target_link_libraries(hello emc-framework)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

