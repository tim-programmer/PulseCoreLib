# Fails if a static library references the compiler's soft-float runtime.
#
#   cmake -DNM=<path to nm> -DLIB=<path to .a> -P PclCheckNoFloat.cmake
#
# Integer helpers (__aeabi_idiv, __aeabi_lmul, __aeabi_ldivmod, ...) are fine;
# only float/double arithmetic, comparison and conversion helpers are rejected.

execute_process(
    COMMAND ${NM} --undefined-only ${LIB}
    OUTPUT_VARIABLE nm_output
    ERROR_VARIABLE nm_error
    RESULT_VARIABLE nm_result
)
if(NOT nm_result EQUAL 0)
    message(FATAL_ERROR "nm failed on ${LIB}: ${nm_error}")
endif()

string(REPLACE "\n" ";" nm_lines "${nm_output}")
set(offenders "")
foreach(line IN LISTS nm_lines)
    string(STRIP "${line}" line)
    if(NOT line MATCHES "^U (.+)$")
        continue()
    endif()
    set(sym "${CMAKE_MATCH_1}")
    # ARM EABI helpers: __aeabi_fadd, __aeabi_dmul, __aeabi_cfcmple, __aeabi_i2f, ...
    # libgcc helpers:   __mulsf3, __floatsisf, __fixsfsi, __extendsfdf2, ...
    if(sym MATCHES "^__aeabi_(f|d|cf|cd|i2f|i2d|ui2f|ui2d|l2f|l2d|ul2f|ul2d)"
       OR sym MATCHES "^__[a-z]+(sf|df)[a-z0-9]*$")
        list(APPEND offenders "${sym}")
    endif()
endforeach()

if(offenders)
    list(REMOVE_DUPLICATES offenders)
    string(REPLACE ";" "\n  " offender_list "${offenders}")
    message(FATAL_ERROR
        "${LIB} pulls in the soft-float runtime (float/double used in a fixed-point build):\n"
        "  ${offender_list}")
endif()
