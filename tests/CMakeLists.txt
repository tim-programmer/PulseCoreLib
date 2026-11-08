find_library(PCL_MATH_LIBRARY m)

# One executable per test source; each is a single translation unit that includes
# pcl_test.h (which holds the harness state as file-scope statics).
function(pcl_add_test name)
    add_executable(${name} ${name}.c)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${name} PRIVATE pcl pcl_warnings)
    if(PCL_MATH_LIBRARY)
        target_link_libraries(${name} PRIVATE ${PCL_MATH_LIBRARY})
    endif()
    set_target_properties(${name} PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS OFF
    )
    add_test(NAME ${name} COMMAND ${name})
endfunction()

pcl_add_test(test_types)
pcl_add_test(test_tables)
target_include_directories(test_tables PRIVATE ${PROJECT_SOURCE_DIR}/src)
pcl_add_test(test_math)
pcl_add_test(test_smooth)
pcl_add_test(test_biquad)
pcl_add_test(test_clock)
pcl_add_test(test_arp)
pcl_add_test(test_seq)
pcl_add_test(test_event_chain)
pcl_add_test(test_biquad_internal)
target_include_directories(test_biquad_internal PRIVATE ${PROJECT_SOURCE_DIR}/src)
pcl_add_test(test_svf)
pcl_add_test(test_svf_internal)
target_include_directories(test_svf_internal PRIVATE ${PROJECT_SOURCE_DIR}/src)
pcl_add_test(test_vca)
pcl_add_test(test_osc)
pcl_add_test(test_noise)
pcl_add_test(test_lfo)
pcl_add_test(test_adsr)
