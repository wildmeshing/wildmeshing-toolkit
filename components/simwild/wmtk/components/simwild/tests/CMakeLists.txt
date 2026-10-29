if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

set(SRC_FILES
    test_simwild_energies.cpp
    test_expression_parser.cpp
    test_simwild_swap_tags.cpp
    test_wild_conformance.cpp
    test_embed_surface_components.cpp
    test_simplify_boundary.cpp
    test_late_feature_tracking.cpp
    test_order2_tube.cpp
    )

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk::triwild wmtk::tetwild)
# The history tests rebuild the map with the tracking component. It is added after this component, so its
# option may not be set yet; unset, it defaults to on.
if(NOT DEFINED WMTK_ENABLE_COMPONENT_tracking OR WMTK_ENABLE_COMPONENT_tracking)
    target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE test_simwild_history.cpp test_simwild_tracking.cpp)
    target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk::tracking)
endif()
# test_simplify_boundary.cpp builds its parameters the way the application does, defaults
# injected from the spec.
target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE jse::jse wmtk_${COMPONENT_NAME}_spec)
