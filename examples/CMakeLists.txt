add_executable(sweep_wav sweep_wav.c)
target_link_libraries(sweep_wav PRIVATE pcl pcl_warnings)
set_target_properties(sweep_wav PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

find_library(PCL_MATH_LIBRARY m)
if(PCL_MATH_LIBRARY)
    target_link_libraries(sweep_wav PRIVATE ${PCL_MATH_LIBRARY})
endif()

# A short render keeps the example from rotting: it must build and run in every sample type.
if(PCL_BUILD_TESTS)
    add_test(NAME example_sweep_wav
        COMMAND sweep_wav ${CMAKE_CURRENT_BINARY_DIR}/smoke.wav saw 1)
endif()
