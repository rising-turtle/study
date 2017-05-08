
INCLUDE_DIRECTORIES(${EIGEN} ${MRPT_DIRS})
# MESSAGE(STATUS, "MRPT_DIRS: !! ${MRPT_DIRS}")

SET( ICP_WARPPER_SRCS
	ZHIcp_Warpper.cpp
)


LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/lib ${MRPT_LIB})

SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
ADD_LIBRARY(zhicp SHARED ${ICP_WARPPER_SRCS})
target_link_libraries(zhicp ${MRPT_LIBS})

# Add the required libraries for linking:
ADD_EXECUTABLE(test_icp test.cpp)
TARGET_LINK_LIBRARIES(test_icp ${MRPT_LIBS})

ADD_EXECUTABLE(test_warpper testWarp.cpp)
TARGET_LINK_LIBRARIES(test_warpper zhicp) # ${MRPT_LIBS})

ADD_EXECUTABLE(test_fr079 testfr079.cpp)
target_link_libraries(test_fr079 zhicp)
