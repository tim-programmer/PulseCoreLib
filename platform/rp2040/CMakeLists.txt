# Benchmark firmware for an RP2040 board (Raspberry Pi Pico: Cortex-M0+, no FPU, 125 MHz).
#
# A standalone CMake project, because the Pico SDK has to be imported before project(). It builds
# the library sources directly with the project's warning flags, plus platform/bench, and talks to
# the host over USB serial (stdio_usb). Driven by scripts/rp2040_bench.py:
#
#     cmake -S platform/rp2040 -B build-rp2040-q15 -DPCL_SAMPLE_TYPE=Q15
#     cmake --build build-rp2040-q15
#
# Needs the Pico SDK (PICO_SDK_PATH, e.g. /usr/share/pico-sdk) and arm-none-eabi-gcc.
cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "Set PICO_SDK_PATH (environment or -D) to the Pico SDK, e.g. /usr/share/pico-sdk")
endif()
set(PICO_BOARD pico CACHE STRING "Pico SDK board")
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Same optimisation as the MCXN947 firmware, so the two boards compare like for like.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "" FORCE)

project(pcl_bench_rp2040 C CXX ASM)
pico_sdk_init()

set(PCL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(PCL_SAMPLE_TYPE "Q15" CACHE STRING "Sample type: FLOAT, Q15 or Q31")
if(NOT PCL_SAMPLE_TYPE MATCHES "^(FLOAT|Q15|Q31)$")
    message(FATAL_ERROR "PCL_SAMPLE_TYPE must be FLOAT, Q15 or Q31 (got '${PCL_SAMPLE_TYPE}')")
endif()

# The library, with the flags it is held to everywhere (CLAUDE.md hard rule 4).
file(GLOB PCL_SOURCES CONFIGURE_DEPENDS ${PCL_ROOT}/src/*/*.c)
add_library(pcl STATIC ${PCL_SOURCES})
target_include_directories(pcl PUBLIC ${PCL_ROOT}/include PRIVATE ${PCL_ROOT}/src)
target_compile_definitions(pcl PUBLIC PCL_SAMPLE_TYPE=PCL_${PCL_SAMPLE_TYPE})
target_compile_options(pcl PRIVATE -Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow
    -Wdouble-promotion -Wcast-qual -ffunction-sections -fdata-sections)
set_target_properties(pcl PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

add_executable(pcl_bench_rp2040
    main.c
    ${PCL_ROOT}/platform/bench/pcl_bench.c
)
target_include_directories(pcl_bench_rp2040 PRIVATE ${PCL_ROOT}/platform/bench)
set_source_files_properties(main.c ${PCL_ROOT}/platform/bench/pcl_bench.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")
target_link_libraries(pcl_bench_rp2040 PRIVATE pcl pico_stdlib hardware_sync)

# USB serial only (the Pico's micro-USB port); binary output, so no CR/LF translation.
pico_enable_stdio_usb(pcl_bench_rp2040 1)
pico_enable_stdio_uart(pcl_bench_rp2040 0)
target_compile_definitions(pcl_bench_rp2040 PRIVATE PICO_STDIO_DEFAULT_CRLF=0)
pico_add_extra_outputs(pcl_bench_rp2040)
