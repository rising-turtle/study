cmake_minimum_required(VERSION 2.8.3)
project(tryQuatJacobian)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -w ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

include_directories("/usr/include/eigen3")

add_subdirectory(src bin)
 




