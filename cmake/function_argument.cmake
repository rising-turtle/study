function(use_llvm F)
message("ARGC=\"${ARGC}\"") # N number of parameters
message("ARGN=\"${ARGN}\"") # list of parameters not specified
message("ARGV=\"${ARGV}\"") # list of all parameters 
message("ARGV0=\"${ARGV0}\"") # All parameters [0]
message("ARGV1=\"${ARGV1}\"") # All parameters [1]
endfunction()

add_custom_target(foo
COMMAND ls)

use_llvm(/foo core;bitwriter;cc b)
