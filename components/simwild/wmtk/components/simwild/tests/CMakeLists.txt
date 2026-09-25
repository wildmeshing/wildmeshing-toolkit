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

# The tests of the polyfem_helpers sources, under the same gate those sources are built with.
# test_polyfem_in_process.cpp also drives the polyfem_ops entry point, hence the link to that
# component; polyfem itself comes through wmtk::simwild, which links it PUBLIC.
if(WMTK_WITH_POLYFEM)
    set(POLYFEM_HELPERS_SRC_FILES
        test_polyfem_link.cpp
        test_python_repr.cpp
        test_interface_selection.cpp
        test_numpy_compat.cpp
        test_mesh_reduction.cpp
        test_polyfem_json.cpp
        test_polyfem_runner.cpp
        test_polyfem_in_process.cpp
        )

    target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${POLYFEM_HELPERS_SRC_FILES})
    target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk::polyfem_ops)
endif()
