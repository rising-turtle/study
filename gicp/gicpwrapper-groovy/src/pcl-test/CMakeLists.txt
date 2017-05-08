add_executable(demo_vis test_viz.cpp)
target_link_libraries(demo_vis pcl_common pcl_io pcl_kdtree pcl_visualization ${VTK_LIBRARIES})

