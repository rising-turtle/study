function(use_llvm TARGET)
message("ARGC=\"${ARGC}\"")
message("ARGN=\"${ARGN}\"")
message("ARGV=\"${ARGV}\"")
message("ARGV0=\"${ARGV0}\"")
message("ARGV1=\"${ARGV1}\"")
endfunction()

add_custom_target(foo
COMMAND ls)

use_llvm(foo core bitwriter)
