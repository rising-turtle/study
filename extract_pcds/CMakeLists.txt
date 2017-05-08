project(EXTRACT_PCDS)

set(the_name extract_pcds)
cmake_minimum_required(VERSION 2.8)

set(PCL_DIR /home/iotrobot/Src/pcl-git/build)
find_package(PCL 1.7 REQUIRED)
# message("PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
# message("PCL_LIBRARIES: ${PCL_LIBRARIES}")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# find_package(CUDA REQUIRED)
# message("CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")
# include_directories(${CUDA_INCLUDE_DIRS})

# OpenNI 
# message("CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)
# message("CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}")

find_package(OpenNI REQUIRED)
link_directories(${OpenNI_LIBRARY_DIRS})
# message("OpenNI_LIBRARY_DIRS: ${OpenNI_LIBRARY_DIRS}")

message("OPENNI_LIBRARY: ${OPENNI_LIBRARY}")

include_directories(src)
# FILE(GLOB srcs "src/pcl_dec* src/extract_pcd*")
set(srcs src/pcl_dec.cpp 
         src/extract_pcds.cpp
        )
message("srcs : ${srcs}")
add_library(${the_name} ${srcs})
target_link_libraries(${the_name} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENNI_LIBRARY})

add_executable(test1 src/main.cpp)
target_link_libraries(test1 ${the_name}) #${OpenCV_LIBRARIES} ${OPENNI_LIBRARY})

add_executable(test2 src/main2.cpp)
target_link_libraries(test2 ${the_name})

