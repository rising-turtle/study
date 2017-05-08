
project(gtsam_test)

 set(GTSAM_ROOT /home/davidz/work/3rdlibs/gtsam-3.1.0/build)
#  set(GTSAM_ROOT /home/davidz/work/3rdlibs/gtsam-4.0/build)
 find_package(GTSAM REQUIRED 
   PATHS ${GTSAM_ROOT}) 

 # include_directories(${GTSAM_INCLUDE_DIR} ${GTSAM_ROOT} ${GTSAM_ROOT}/../  ${GTSAM_ROOT}/../gtsam/3rdparty/Eigen)
include_directories(${GTSAM_INCLUDE_DIR} ${GTSAM_ROOT} /usr/include/eigen3)
message(STATUS "GTSAM_INCLUDE_DIR: ${GTSAM_INCLUDE_DIR}" )
# link_directories(${GTSAM_Root}/gtsam /home/davidz/work/3rdlibs/gtsam-3.1.0/install/usr/local/lib)
link_directories(${GTSAM_ROOT}/gtsam )


add_executable(test_imu testImuFactor.cpp)
target_link_libraries(test_imu gtsam CppUnitLite)

add_executable(test_trans testTrans.cpp)
target_link_libraries(test_trans gtsam)

# add_executable(imugps_example imuGPS_example.cpp)
# target_link_libraries(imugps_example gtsam CppUnitLite)


