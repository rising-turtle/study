

macro(FIND_OpenCV LIB_DIR VERSION_STR)
    message("LIB_DIR : ${LIB_DIR} VERSION_STR: ${VERSION_STR}")
    message("ARGC = ${ARGC}")
    message("ARGN = ${ARGN}")
    message("ARGV: ${ARGV}")
    message("ARGV0 = ${ARGV0} ARGV1= ${ARGV1} ARGV2 = ${ARGV2} ARGV3 = ${ARGV3}" )
    set(OpenCV_LIBS "")
    foreach(__module ${ARGN})
	set(OpenCV_MODULE_LIB "${LIB_DIR}/lib${__module}.so.${VERSION_STR}")
	# set(OpenCV_LIBS "${OpenCV_LIBS};${OpenCV_MODULE_LIB}")
	list(APPEND OpenCV_LIBS "${OpenCV_MODULE_LIB}")
    endforeach()
endmacro()

set(OpenCV_DIR "/home/davidz/work/3rdlibs/opencv/build")

find_package(OpenCV 3.4 REQUIRED)

message("OpenCV_LIBS: ${OpenCV_LIBS}")

set(LIB_DIR "${OpenCV_DIR}/lib")
set(VERSION_STR "3.4")
set(MODULES ${OpenCV_LIBS})
FIND_OpenCV(${LIB_DIR} ${VERSION_STR} ${MODULES})


message("OpenCV_LIBS: ${OpenCV_LIBS}")
# message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")



