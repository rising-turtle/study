project(sleepingta_PROJ)

cmake_minimum_required(VERSION 2.8.0)

set(CMAKE_C_FLAGS "-O3 -w -g -ggdb") # set c compiler flags 

add_executable(sleepingta main.c hangout.c help_student.c ta.c student.c)
target_link_libraries(sleepingta pthread)

