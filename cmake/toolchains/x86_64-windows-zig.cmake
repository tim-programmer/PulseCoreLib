# Windows x86-64 cross-build from Linux/macOS, using zig's bundled clang and MinGW-w64 (UCRT)
# headers and libraries, so no Windows SDK is needed. Tests run under Wine via
# CMAKE_CROSSCOMPILING_EMULATOR when it is installed.
#
# Used by the win-* presets. This checks the GCC/Clang flavour of Windows; MSVC itself
# (/W4 /WX) is covered by the GitHub Actions workflow in .github/workflows/windows.yml.
#
# Wine notes: its ucrtbase.dll lacks a few C99 functions (cabs, for one), so tests must not
# depend on them. Set WINEPREFIX to keep the Wine state out of ~/.wine, and WINEDEBUG=-all
# to quieten it.

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR AMD64)

set(_pcl_zig_dir ${CMAKE_CURRENT_LIST_DIR}/zig)
set(CMAKE_C_COMPILER ${_pcl_zig_dir}/zig-cc)
set(CMAKE_AR ${_pcl_zig_dir}/zig-ar CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB ${_pcl_zig_dir}/zig-ranlib CACHE FILEPATH "" FORCE)

# Search the target environment only for headers and libraries, never the host's
# (otherwise find_library(m) picks up the host libm.a and the link fails).
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

find_program(PCL_WINE wine)
if(PCL_WINE)
    set(CMAKE_CROSSCOMPILING_EMULATOR ${PCL_WINE})
endif()
