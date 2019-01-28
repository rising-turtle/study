project(test_ceres)

cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -std=c++11 -w -g -O3 -fpermissive -mno-avx -mno-avx2")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# set(Ceres_ROOT "/home/davidz/work/3rdlib/ceres-solver/ceres-bin")
find_package(Ceres REQUIRED)
# set(CERES_INCLUDE_DIRS "/home/davidz/work/3rdlib/ceres-solver/include")
include_directories(${CERES_INCLUDE_DIRS} "/usr/include/eigen3" "./src")

message(STATUS "Ceres: ${CERES_INCLUDE_DIRS} CERES_LIBRARIES: ${CERES_LIBRARIES}" )

link_libraries(${CERES_LIBRARIES})
# link_libraries("/home/davidz/work/3rdlib/ceres-solver/ceres-bin/lib/libceres.a")
add_executable(test_ceres src/projection_quat.cpp src/test_projection.cpp)
