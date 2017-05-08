cmake_minimum_required(VERSION 2.4.6)

 if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
 endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)

set(OPENNI2_INCLUDE_DIR "C:/Program Files/OpenNI2/Include")
set(OPENNI2_LIBRARY_DIR "C:/Program Files/OpenNI2/Lib")
set(OPENNI2_LIBS OpenNI2)

include_directories(${OPENNI2_INCLUDE_DIR})    
link_directories(${OPENNI2_LIBRARY_DIR})    

#set(OPENCV_INCLUDE_DIR "D:/WinSrc/opencv/sources/build_d/install/include")
#set(OPENCV_LIBRARY_DIR "D:/WinSrc/opencv/sources/build_d/install/x86/vc9/lib")
set(OpenCV_DIR "D:/Src/opencv2.7")

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIR})    
link_directories(${OpenCV_LIBRARY_DIR}) 

#Local Main
add_executable( openni2_recorder src/openni2_recorder.cpp )
TARGET_LINK_LIBRARIES(openni2_recorder ${OpenCV_LIBS} ${OPENNI2_LIBS})

add_executable( main_mopenni2 src/main_mopenni2.cpp src/mopenni2.cpp )
TARGET_LINK_LIBRARIES(main_mopenni2 ${OpenCV_LIBS} ${OPENNI2_LIBS})

