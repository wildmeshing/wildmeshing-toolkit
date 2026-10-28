if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

# Well past ctest's 1500 s default, which this suite straddles on Windows Debug (CI: one green
# run at 1343.9 s, a 1500.14 s timeout on every other push of this branch; every other platform
# far under). The integration tests set their budget the same way (tests/CMakeLists.txt, 7200).
if(TEST wmtk_test_${COMPONENT_NAME})
    set_tests_properties(wmtk_test_${COMPONENT_NAME} PROPERTIES TIMEOUT 3600)
endif()

set(SRC_FILES
    test_topological_offset.cpp
    test_offset_potential.cpp
    test_exact_band_integral.cpp
    test_exact_band_front.cpp
    exact_band_integral_data.hpp
    exact_band_max_data.hpp
    exact_band_parallel_data.hpp
    exact_band_side_data.hpp
    exact_band_front_data.hpp
    )

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
# The spec's defaults, so a test can build Parameters as the app does.
target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk_${COMPONENT_NAME}_spec)
