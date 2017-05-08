

cmake_minimum_required(VERSION 2.8.3)
project(stereo_match)

set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -w -std=c++0x -fpermissive")

#################
#### OpenCV #####
#################
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/thirdParty/libelas/src")
add_executable(test_match src/test_stereo_match.cpp)
target_link_libraries(test_match ${PROJECT_SOURCE_DIR}/thirdParty/libelas/libstereo.so ${OpenCV_LIBRARIES})





