

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
message("STATUS OpenCV_INCLUDE_DIRS : ${OpenCV_INCLUDE_DIRS}")

add_executable(write_video video-write.cpp)
target_link_libraries(write_video ${OpenCV_LIBS})

add_executable(read_video video-input-psnr-ssim.cpp)
target_link_libraries(read_video ${OpenCV_LIBS})

add_executable(rw_video video_read_write.cpp)
target_link_libraries(rw_video ${OpenCV_LIBS})


