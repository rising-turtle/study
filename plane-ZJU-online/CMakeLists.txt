cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PlaneSegmentation)

# set(PCL_DIR "/home/davidz/work/PCL-1.6.0-Source/build")
set(PCL_DIR "/home/davidz/work/3rdproject/pcl-1.7.0/build")
find_package(PCL 1.7)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PlaneSegmentation ./src/plane_segmentation.cpp ./src/planeSeg_MAIN.cpp)
target_link_libraries (PlaneSegmentation ${PCL_LIBRARIES})
