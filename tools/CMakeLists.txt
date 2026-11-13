add_executable(gen_tables gen_tables.c)
target_link_libraries(gen_tables PRIVATE pcl_warnings)
set_target_properties(gen_tables PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

if(NOT WIN32) # the maths functions are in the C runtime on Windows
    find_library(PCL_MATH_LIBRARY m)
endif()
if(PCL_MATH_LIBRARY)
    target_link_libraries(gen_tables PRIVATE ${PCL_MATH_LIBRARY})
endif()

# Host reference output of the benchmark cases, compared against the MCX N947 firmware.
add_executable(bench_ref bench_ref.c ${PROJECT_SOURCE_DIR}/platform/bench/pcl_bench.c)
target_include_directories(bench_ref PRIVATE ${PROJECT_SOURCE_DIR}/platform/bench)
target_link_libraries(bench_ref PRIVATE pcl pcl_warnings)
set_target_properties(bench_ref PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

# Listening-test renderer (WAV files). Run as a short smoke test so it cannot rot.
add_executable(render render.c)
target_link_libraries(render PRIVATE pcl pcl_warnings)
set_target_properties(render PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(PCL_MATH_LIBRARY)
    target_link_libraries(render PRIVATE ${PCL_MATH_LIBRARY})
endif()
# Plays a melody from a WAV file through the sampler (16-bit PCM; reader in wav_read.c).
add_executable(play_sample play_sample.c wav_read.c)
target_link_libraries(play_sample PRIVATE pcl pcl_warnings)
set_target_properties(play_sample PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

if(PCL_BUILD_TESTS)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/render_smoke)
    add_test(NAME tool_render_noise
        COMMAND render noise ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.2)
    add_test(NAME tool_render_lfo
        COMMAND render lfo ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.2)
    add_test(NAME tool_render_adsr
        COMMAND render adsr ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.2)
    add_test(NAME tool_render_svf
        COMMAND render svf ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.2)
    add_test(NAME tool_render_biquad
        COMMAND render biquad ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.2)
    add_test(NAME tool_render_events
        COMMAND render events ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.5)
    add_test(NAME tool_render_ladder
        COMMAND render ladder ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.2)
    add_test(NAME tool_render_delay
        COMMAND render delay ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.2)
    add_test(NAME tool_render_sampler
        COMMAND render sampler ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.5)
    add_test(NAME tool_render_reverb
        COMMAND render reverb ${CMAKE_CURRENT_BINARY_DIR}/render_smoke 0.3)
    # play_sample reads a file the render step wrote (exercises the WAV reader end to end).
    set_tests_properties(tool_render_sampler PROPERTIES FIXTURES_SETUP sampler_wav)
    add_test(NAME tool_play_sample
        COMMAND play_sample ${CMAKE_CURRENT_BINARY_DIR}/render_smoke/sampler_melody.wav
                ${CMAKE_CURRENT_BINARY_DIR}/render_smoke/play_sample_out.wav 60)
    set_tests_properties(tool_play_sample PROPERTIES FIXTURES_REQUIRED sampler_wav)
endif()
