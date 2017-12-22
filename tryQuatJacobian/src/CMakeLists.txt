
project(tryQuatJacobian)

set(LSD_SLAM_DIR /home/davidz/work/ros/indigo/src/lsd_slam_ext/lsd_slam_core)
include_directories(${LSD_SLAM_DIR}/thirdparty/Sophus/sophus)

add_executable(quatJacobian tryQuatJacobian.cpp utility.cpp)
