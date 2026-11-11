add_library(pcl STATIC
    core/pcl_math.c
    core/pcl_smooth.c
    core/pcl_tables.c
    event/pcl_arp.c
    event/pcl_clock.c
    event/pcl_seq.c
    filter/pcl_biquad.c
    filter/pcl_ladder.c
    filter/pcl_svf.c
    fx/pcl_delay.c
    mod/pcl_adsr.c
    mod/pcl_lfo.c
    mod/pcl_vca.c
    osc/pcl_noise.c
    osc/pcl_osc.c
    osc/pcl_unison.c
    sampler/pcl_sampler.c
)
add_library(pcl::pcl ALIAS pcl)

target_include_directories(pcl PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(pcl PRIVATE ${PROJECT_SOURCE_DIR}/src)

# The sample type is part of the ABI, so consumers must see the same definition.
target_compile_definitions(pcl PUBLIC PCL_SAMPLE_TYPE=PCL_${PCL_SAMPLE_TYPE})
if(PCL_ALLOW_FLOAT_INIT)
    target_compile_definitions(pcl PUBLIC PCL_ALLOW_FLOAT_INIT)
endif()

set_target_properties(pcl PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)
target_link_libraries(pcl PRIVATE pcl_warnings pcl_strict_warnings)

# On a soft-float target, any float arithmetic shows up as a call into the compiler
# runtime (__aeabi_fmul, __addsf3, ...). Fail the build if a fixed-point library
# references one (CLAUDE.md hard rule 5). On hosts with an FPU float ops are inline
# instructions, so this check is only meaningful when cross-compiling for a soft-float part;
# the FPU presets (cm33-*) switch it off with -DPCL_CHECK_NO_FLOAT=OFF.
option(PCL_CHECK_NO_FLOAT "Fail the build if a fixed-point library references the soft-float runtime" ON)
if(CMAKE_CROSSCOMPILING AND PCL_CHECK_NO_FLOAT AND NOT PCL_SAMPLE_TYPE STREQUAL "FLOAT"
   AND NOT PCL_ALLOW_FLOAT_INIT)
    if(NOT CMAKE_NM)
        message(FATAL_ERROR "CMAKE_NM not found; cannot check the library for soft-float references")
    endif()
    add_custom_command(TARGET pcl POST_BUILD
        COMMAND ${CMAKE_COMMAND}
            -DNM=${CMAKE_NM}
            -DLIB=$<TARGET_FILE:pcl>
            -P ${PROJECT_SOURCE_DIR}/cmake/PclCheckNoFloat.cmake
        COMMENT "Checking libpcl (${PCL_SAMPLE_TYPE}) for soft-float runtime references"
        VERBATIM
    )
endif()
