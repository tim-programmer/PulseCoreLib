cmake_minimum_required(VERSION 3.16)

project(PulseCoreLib VERSION 0.1.0 LANGUAGES C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- Options -----------------------------------------------------------------

set(PCL_SAMPLE_TYPE "FLOAT" CACHE STRING "Sample type: FLOAT, Q15 or Q31")
set_property(CACHE PCL_SAMPLE_TYPE PROPERTY STRINGS FLOAT Q15 Q31)
if(NOT PCL_SAMPLE_TYPE MATCHES "^(FLOAT|Q15|Q31)$")
    message(FATAL_ERROR "PCL_SAMPLE_TYPE must be FLOAT, Q15 or Q31 (got '${PCL_SAMPLE_TYPE}')")
endif()

option(PCL_ALLOW_FLOAT_INIT
    "Let init/config functions use float in Q15/Q31 builds (never allowed in process paths)" OFF)
option(PCL_BUILD_TESTS "Build the host-side unit tests" ON)
option(PCL_BUILD_EXAMPLES "Build the host-side example programs" ON)
option(PCL_BUILD_TOOLS "Build the host-side utilities (table generator, benchmark reference)" ON)
option(PCL_BUILD_MCXN947 "Build the FRDM-MCXN947 benchmark firmware (needs the NXP SDK in third_party/)" OFF)

# ---- Warnings ----------------------------------------------------------------
# Every target in the project is built warning-clean (CLAUDE.md hard rule 4).
# The library additionally gets the stricter set that guards fixed-point code.

add_library(pcl_warnings INTERFACE)
add_library(pcl_strict_warnings INTERFACE)
if(MSVC)
    target_compile_options(pcl_warnings INTERFACE /W4 /WX)
    # fopen and friends are standard C99; MSVC's C4996 "use fopen_s" warning would fail /WX.
    # Only the host tools and tests do file I/O; the core has none (hard rule 6).
    target_compile_definitions(pcl_warnings INTERFACE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(pcl_warnings INTERFACE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(pcl_strict_warnings INTERFACE
        -Wconversion -Wshadow -Wdouble-promotion -Wcast-qual)
endif()

# ---- Library and tests -------------------------------------------------------

add_subdirectory(src)

if(PCL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(PCL_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(PCL_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(PCL_BUILD_MCXN947)
    add_subdirectory(platform/mcxn947)
endif()
