# Oscillator benchmark firmware for the NXP FRDM-MCXN947 (Cortex-M33 core 0).
# Needs the SDK pieces fetched by scripts/fetch_mcuxsdk.sh into third_party/ and the
# cm33 toolchain; configure with one of the mcxn947-* presets.
enable_language(ASM)

set(SDK ${PROJECT_SOURCE_DIR}/third_party)
set(DEV ${SDK}/mcux-devices-mcx/MCXN/MCXN947)
set(DRV ${SDK}/mcuxsdk-core/drivers)
set(BRD ${SDK}/mcuxsdk-examples/_boards/frdmmcxn947)

if(NOT EXISTS ${DEV}/system_MCXN947_cm33_core0.c)
    message(FATAL_ERROR "NXP SDK not found in third_party/. Run scripts/fetch_mcuxsdk.sh first.")
endif()

set(SDK_SOURCES
    ${DEV}/gcc/startup_MCXN947_cm33_core0.S
    ${DEV}/system_MCXN947_cm33_core0.c
    ${DEV}/drivers/fsl_clock.c
    ${DEV}/drivers/fsl_reset.c
    ${DRV}/common/fsl_common.c
    ${DRV}/common/fsl_common_arm.c
    ${DRV}/lpflexcomm/fsl_lpflexcomm.c
    ${DRV}/lpuart/fsl_lpuart.c
    ${DRV}/mcx_spc/fsl_spc.c
    ${BRD}/clock_config.c
    ${BRD}/demo_apps/hello_world/pin_mux.c
)
# Vendor code is not held to this project's warning policy; ours is.
set_source_files_properties(${SDK_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")

add_executable(pcl_bench_mcxn947
    main.c
    ${PROJECT_SOURCE_DIR}/platform/bench/pcl_bench.c
    ${SDK_SOURCES}
)
set_target_properties(pcl_bench_mcxn947 PROPERTIES
    SUFFIX .elf C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

target_include_directories(pcl_bench_mcxn947 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/platform/bench)
# SYSTEM: vendor headers are not checked against our warning flags either.
target_include_directories(pcl_bench_mcxn947 SYSTEM PRIVATE
    ${DEV} ${DEV}/drivers ${SDK}/mcux-devices-mcx/MCXN/periph ${BRD} ${BRD}/demo_apps/hello_world
    ${DRV}/common ${DRV}/lpflexcomm ${DRV}/lpuart ${DRV}/mcx_spc ${DRV}/port
    ${SDK}/mcu-sdk-cmsis/Core/Include)

target_compile_definitions(pcl_bench_mcxn947 PRIVATE
    CPU_MCXN947VDF_cm33_core0 SDK_DEBUGCONSOLE=0 __STARTUP_CLEAR_BSS)

# Switch the flash cache off to see how much of the cycle count is flash fetch time.
option(PCL_MCXN947_DISABLE_LPCAC "Disable the MCX N flash cache (LPCAC) in the benchmark" OFF)
if(PCL_MCXN947_DISABLE_LPCAC)
    target_compile_definitions(pcl_bench_mcxn947 PRIVATE BENCH_DISABLE_LPCAC)
endif()

target_link_libraries(pcl_bench_mcxn947 PRIVATE pcl)
set_source_files_properties(main.c ${PROJECT_SOURCE_DIR}/platform/bench/pcl_bench.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")

target_link_options(pcl_bench_mcxn947 PRIVATE
    -T${DEV}/gcc/MCXN947_cm33_core0_flash.ld
    -Wl,--defsym=__stack_size__=0x2000
    -Wl,--gc-sections
    -Wl,--no-warnings
    -Wl,-Map=$<TARGET_FILE_DIR:pcl_bench_mcxn947>/pcl_bench_mcxn947.map
    --specs=nano.specs --specs=nosys.specs)

find_program(PCL_ARM_SIZE NAMES arm-none-eabi-size)
add_custom_command(TARGET pcl_bench_mcxn947 POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:pcl_bench_mcxn947>
            $<TARGET_FILE_DIR:pcl_bench_mcxn947>/pcl_bench_mcxn947.bin
    VERBATIM)
if(PCL_ARM_SIZE)
    add_custom_command(TARGET pcl_bench_mcxn947 POST_BUILD
        COMMAND ${PCL_ARM_SIZE} $<TARGET_FILE:pcl_bench_mcxn947> VERBATIM)
endif()
