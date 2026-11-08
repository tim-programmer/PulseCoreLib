# Cortex-M0+ (no FPU, no hardware divide) compile check with the GNU Arm toolchain.
# Used by the cm0plus-* presets; it builds the library only, nothing is linked.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)

# There is no board support to link a test executable against.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS_INIT
    "-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -ffunction-sections -fdata-sections")
