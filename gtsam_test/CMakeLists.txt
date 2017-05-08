

project(gtsam_test)

cmake_minimum_required(VERSION 2.6.0)

set(CMAKE_CXX_FLAGS "-ggdb -O3 -w")

add_subdirectory(src bin)
