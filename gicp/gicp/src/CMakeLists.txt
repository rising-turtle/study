# project(gicp)
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-w -ggdb -g -fopenmp")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/include /usr/include)
link_directories(${CMAKE_SOURCE_DIR}/lib /usr/lib)
# file(GLOB SRCS ./*.cpp)
set(SRCS bfgs_funcs.cpp scan.cpp transform.cpp gicp.cpp optimize.cpp)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -fpermissive)
add_library(gicp SHARED ${SRCS})
target_link_libraries(gicp ANN gsl gslcblas)

add_executable(test_gicp test_gicp.cpp)
target_link_libraries(test_gicp gicp ANN gsl gslcblas boost_program_options boost_system)

add_executable(scan2ascii scan2ascii.cpp)
target_link_libraries(scan2ascii gicp ANN gsl gslcblas boost_program_options boost_system)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DOMP_ENABLED)
endif(OPENMP_FOUND)

# QT4
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QT3SUPPORT TRUE)
# ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})
include_directories(${Qt4_INCLUDE_DIRS})
set(QT_LIBS GL GLU ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY})

# EIGEN
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# flann
set(FLANN_INCLUDE /home/davidz/work/3rdproject/flann-1.8.4-src/src/cpp)
set(FLANN_LIB_DIR /home/davidz/work/3rdproject/flann-1.8.4-src/lib)
set(FLANN_LIBS flann)
# include_directories(${FLANN_INCLUDE})
link_directories(${FLANN_LIB_DIR})

# PCL implement
set(PCL_INCLUDE /usr/local/include/pcl-1.6)
set(PCL_LIB_DIR /usr/local/lib)
set(PCL_LIBS pcl_common pcl_features pcl_filters pcl_geometry pcl_io pcl_kdtree pcl_keypoints pcl_octree pcl_registration pcl_sample_consensus pcl_search pcl_segmentation pcl_surface pcl_tracking pcl_visualization )
include_directories(${PCL_INCLUDE})
link_directories(${PCL_LIB_DIR})
#  test_flann_ori.cpp
set(SRCS2 main.cpp timestamp.cpp PGicp.cpp globaldef.cpp KDTWrapper.cpp test_flann.cpp test_flann_ori.cpp)
add_executable(main ${SRCS2})
target_link_libraries(main gicp ANN gsl gslcblas ${PCL_LIBS} ${QT_LIBS} ${FLANN_LIBS})
# add_executable(test_registration test_registration.cpp)
# target_link_libraries(test_registration ${PCL_LIBS})





