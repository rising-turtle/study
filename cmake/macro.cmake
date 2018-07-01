
macro(MSG)

foreach(__msg ${ARGN})
    message(STATUS "${__msg} : ${${__msg}}")
endforeach()

endmacro()
