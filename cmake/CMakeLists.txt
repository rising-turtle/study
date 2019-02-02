project(cmake_test)

cmake_minimum_required(VERSION 3.0)

# macros e.g. MSG 
include(macro.cmake)
include(opencv_test.cmake)

add_executable(opencv_test src/opencv_test.cpp)
target_link_libraries(opencv_test ${OpenCV_LIBS})


# message(STATUS "CMAKE_CURRENT_LIST_FILE : ${CMAKE_CURRENT_LIST_FILE}")
# message(STATUS "PATH: ${PATH}")
# get_filename_component(CURRENT_CONFIG_INSTALL_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
# message(STATUS "CURRENT_CONFIG_INSTALL_DIR: ${CURRENT_CONFIG_INSTALL_DIR}")

# message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}" "What1" ${ARGN})

# MSG(CMAKE_MODULE_PATH)
# a function test for ARGN, ARGC, ARGV, ARGV0, ARGV1
# include(function_argument.cmake)

# get_filename_component(CMAKE_CURRENT_PATH "${CMAKE_CURRENT_LIST_FILE}/" REALPATH)
# get_filename_component(CMAKE_CURRENT_PATH "${CMAKE_CURRENT_LIST_FILE}/" PATH)
# MSG(CMAKE_CURRENT_PATH CMAKE_CURRENT_LIST_FILE)

# CMake search sequence 
# 1. CMAKE_PATH, these variables can be specified by -DVAL=val
# MSG(CMAKE_PREFIX_PATH CMAKE_FRAMEWORK_PATH CMAKE_APPBUNDLE_PATH)

# 2. <package_DIR>
# MSG(Ceres_DIR CMAKE)

# 3. HINTS PATH1 [PATH2 ... ] or PATHS PATH1 [PATH2 ...]
# 4. System environment variables 
# MSG(PATH)

# 5, 6, 7 system paths
# MSG(CMAKE_SYSTEM_PREFIX_PATH CMAKE_SYSTEM_FRAMEWORK_PATH CMAKE_SYSTEM_APPBUNDLE_PATH)

# find_package(Ceres QUIET)

# try to add a new prefix path for find_package
# set(CMAKE_PREFIX_PATH "/home/davidz/work/3rdlibs/okvis/build;${CMAKE_PREFIX_PATH}")
# MSG(CMAKE_PREFIX_PATH)

# find_package(Ceres REQUIRED)

# message(STATUS "CERES_INCLUDE_DIRS: ${CERES_INCLUDE_DIRS}")
# message(STATUS "CERES_LIBRARIES: ${CERES_LIBRARIES}")


# find_package(Qt4 REQUIRED)
# MSG(Qt4_FOUND)

set(PCL_DIR "/home/davidz/work/3rdlib/3rdParty_PCL-1.8/build")
# find_package(PCL REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
# message("STATUS PCL_LIBRARY : ${PCL_LIBRARY}")
# message("STATUS PCL_LIBRARIES: ${PCL_LIBRARIES}")
# message("STATUS PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
set(PCL_LIBRARY_DIRS "${PCL_DIR}/lib")
# link_directories(${PCL_LIBRARY_DIRS})
set(PCL_LIBS_1 pcl_common pcl_io pcl_kdtree pcl_visualization pcl_filters pcl_segmentation pcl_search pcl_surface pcl_features)
MSG(${PCL_LIBS_1})
set(PCL_LIBS "")
foreach(__msg ${PCL_LIBS_1})
    set(PCL_MODULE_LIB "${PCL_LIBRARY_DIRS}/lib${__msg}.so")
    list(APPEND PCL_LIBS "${PCL_MODULE_LIB}")
endforeach()
MSG(${PCL_LIBS})



