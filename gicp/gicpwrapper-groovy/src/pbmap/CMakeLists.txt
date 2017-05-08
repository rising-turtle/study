set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} -fopenmp)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# set(MRPT_ROOT "/home/davidz/work/3rdproject/mrpt-1.0.1/libs")
# set(MRPT_INC_DIR 
#    ${MRPT_ROOT}/base/include 
#    ${MRPT_ROOT}/pbmap/include
#    ${MRPT_ROOT}/graphs/include
#    /home/davidz/work/3rdproject/mrpt-1.0.1/build/include/mrpt-config/unix
#    )
# set(MRPT_LIB_DIR ${PROJECT_SOURCE_DIR}/3rdlibs)
# set(MRPT_LIBS mrpt-pbmap mrpt-base)

# include_directories(${MRPT_INC_DIR})
# link_directories(${MRPT_LIB_DIR})

find_package(MRPT REQUIRED)
set(MRPT_ROOT ${MRPT_LIBS_INCL_DIR})
set(MRPT_INCLUDE_DIRS
    ${MRPT_ROOT}/base/include
    ${MRPT_ROOT}/pbmap/include
    ${MRPT_ROOT}/graphs/include
)
set(MRPT_LIBRARY_DIRS /home/davidz/work/3rdproject/mrpt-1.0.1/build/lib)
set(MRPT_LIBRARIES mrpt-base mrpt-pbmap)
include_directories(${MRPT_INCLUDE_DIRS} ${MRPT_CONFIG_DIR})
link_directories(${MRPT_LIBRARY_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

# add_executable(pbmap_sample test_sample.cpp)
# target_link_libraries(pbmap_sample ${MRPT_LIBS} ${PCL_LIBS} )

set(ROS_LIBS rostime)
link_directories(/opt/ros/groovy/lib)
rosbuild_add_executable(pbmap_sample test_pbmap.cpp ../globaldef.cpp ../FileReader.cpp ../timestamp.cpp)
target_link_libraries(pbmap_sample ${MRPT_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBS} ${GICP})

rosbuild_add_executable(pbmap_test test_sample.cpp)
target_link_libraries(pbmap_test ${MRPT_LIBRARIES} ${PCL_LIBRARIES})

# link_directories(/opt/ros/groovy/lib)
# rosbuild_add_executable(pbmap_viz test_viz.cpp)
add_executable(pbmap_viz test_viz.cpp)
target_link_libraries(pbmap_viz ${MRPT_LIBRARIES} ${ROS_LIBS} ${PCL_LIBRARIES} ${VTK_LIBRARIES} )

