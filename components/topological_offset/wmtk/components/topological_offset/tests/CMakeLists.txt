if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

set(SRC_FILES
    test_topological_offset.cpp
    test_offset_potential.cpp
    test_exact_band_integral.cpp
    exact_band_integral_data.hpp
    exact_band_max_data.hpp
    exact_band_side_data.hpp
    )

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
# The spec's defaults, so a test can build Parameters as the app does.
target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk_${COMPONENT_NAME}_spec)
