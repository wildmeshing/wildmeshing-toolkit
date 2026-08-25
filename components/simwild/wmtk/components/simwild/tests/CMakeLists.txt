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
    )

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk::triwild wmtk::tetwild)
