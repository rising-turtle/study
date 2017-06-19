cmake_minimum_required(VERSION 2.8.7)
project(hello_g2o)

set(G2O_ROOT "/home/davidz/work/3rdlibs/g2o") 
include(cmake/FindG2O.cmake)

include_directories("${G2O_INCLUDE_DIR}" "${G2O_ROOT}/g2o" "${G2O_ROOT}/EXTERNAL/csparse" "/usr/include/eigen3")
link_directories("${G2O_ROOT}/lib")


set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11"
) 

# build live ros node
add_executable(hello_g2o hello.cpp)
target_link_libraries(hello_g2o ${G2O_LIBRARIES})
