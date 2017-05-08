# project(gicp)
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
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

# PCL implement
set(PCL_INCLUDE /usr/local/include/pcl-1.6)
set(PCL_LIB_DIR /usr/local/lib)
set(PCL_LIBS pcl_common pcl_features pcl_filters pcl_geometry pcl_io pcl_kdtree pcl_keypoints pcl_octree pcl_registration pcl_sample_consensus pcl_search pcl_segmentation pcl_surface pcl_tracking pcl_visualization )
include_directories(${PCL_INCLUDE})
link_directories(${PCL_LIB_DIR})
set(SRCS2 main.cpp timestamp.cpp PGicp.cpp globaldef.cpp test_flann.cpp)
add_executable(main ${SRCS2})
target_link_libraries(main gicp ANN gsl gslcblas ${PCL_LIBS} ${QT_LIBS})
# add_executable(test_registration test_registration.cpp)
# target_link_libraries(test_registration ${PCL_LIBS})
