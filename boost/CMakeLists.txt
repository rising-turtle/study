project(BOOST_STUDY)

cmake_minimum_required(VERSION 2.8.0)

SET(CMAKE_CXX_FLAGS "$ENV{CXX_FLAGS} -O1 -w -g -ggdb")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXX_FLAGS} -O0 -w -g -ggdb ")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXX_FLAGS} -O3 -w ")

add_subdirectory(src bin)


