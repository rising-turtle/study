set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} -fopenmp)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# rosbuild_init()
find_package(VTK)
if(${VTK_FOUND})
    message("VTK FOUND!")
    include_directories(${VTK_INCLUDE_DIRS})
    link_directories(${VTK_LIBRARY_DIRS})
else(${VTK_FOUND})
    message("VTK NOT FOUND!")
    include_directories(${VTK_INC_DIR})
    link_directories(/usr/lib)
endif(${VTK_FOUND})

set(MRPT_ROOT "/home/davidz/work/3rdproject/mrpt-1.0.1/libs")
set(MRPT_INC_DIR 
    ${MRPT_ROOT}/base/include 
    ${MRPT_ROOT}/pbmap/include
    ${MRPT_ROOT}/graphs/include
    /home/davidz/work/3rdproject/mrpt-1.0.1/build/include/mrpt-config/unix
    )
set(MRPT_LIB_DIR ${PROJECT_SOURCE_DIR}/3rdlibs)
set(MRPT_LIBS mrpt-pbmap mrpt-base)
include_directories(${MRPT_INC_DIR})
link_directories(${MRPT_LIB_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

# add_executable(pbmap_sample test_sample.cpp)
# target_link_libraries(pbmap_sample ${MRPT_LIBS} ${PCL_LIBS} )

set(ROS_LIBS rostime)
link_directories(/opt/ros/fuerte/lib)
add_executable(pbmap_sample test_pbmap.cpp ../globaldef.cpp ../FileReader.cpp ../timestamp.cpp)
target_link_libraries(pbmap_sample ${MRPT_LIBS} ${PCL_LIBS} ${ROS_LIBS} ${OCTOMAP_LIBS} ${GICP})

 add_executable(pbmap_viz test_viz.cpp)
 target_link_libraries(pbmap_viz ${MRPT_LIBS} ${PCL_LIBS} ${VTK_LIBRARIES})

