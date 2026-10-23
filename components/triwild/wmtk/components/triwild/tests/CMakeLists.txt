if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

set(SRC_FILES
    test_sizing_refine.cpp
    test_simplify_segments.cpp
    test_winding_tags.cpp
    test_feature_points.cpp
    test_free_points.cpp
    test_coarsen_pass.cpp
    test_orientation_2d.cpp
    test_colored_smoothing.cpp
    )

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

# The tracking example rebuilds the map with the tracking component. It is added after this
# component, so its option may not be set yet; unset, it defaults to on.
if(NOT DEFINED WMTK_ENABLE_COMPONENT_tracking OR WMTK_ENABLE_COMPONENT_tracking)
    target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE test_triwild_tracking.cpp)
    target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk::tracking)
endif()
