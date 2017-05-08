

project(robocane_control)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../external)
link_directories(/usr/local/lib)

add_library(robocane_control robocane_control.cpp)
target_link_libraries(robocane_control mraa EposCmd ftd2xx ${catkin_LIBRARIES})

add_executable(test_robocane_control test_robo_control.cpp)
target_link_libraries(test_robocane_control robocane_control)
