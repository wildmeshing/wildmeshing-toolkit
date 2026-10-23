if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

set(SRC_FILES
    test_file_write.cpp
    test_insertion.cpp
    test_feature_tags.cpp
    test_orientation.cpp
    test_operation_smooth.cpp
    test_operations.cpp
    test_surface_swap.cpp
    test_sizing_refine.cpp
    test_coarsen_pass.cpp
    test_colored_smoothing.cpp
    test_runner.cpp
    test_surface_folds.cpp
    )

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
# The history tests rebuild the map with the tracking component. It is added after this component, so its
# option may not be set yet; unset, it defaults to on.
if(NOT DEFINED WMTK_ENABLE_COMPONENT_tracking OR WMTK_ENABLE_COMPONENT_tracking)
    target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE test_tetwild_history.cpp test_tetwild_tracking.cpp)
    target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk::tracking)
endif()