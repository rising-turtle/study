


find_package(Boost REQUIRED) 

include_directories(${Boost_INCLUDE_DIRS})

add_executable(boost_signal2 boost_signal2.cpp)
