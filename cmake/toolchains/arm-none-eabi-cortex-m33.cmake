# Cortex-M33 with the single-precision FPU (NXP MCX N947, LPC55S69, ...), GNU Arm toolchain.
# Used by the cm33-* presets; it builds the library only, nothing is linked.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)

# There is no board support to link a test executable against.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS_INIT
    "-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections")
