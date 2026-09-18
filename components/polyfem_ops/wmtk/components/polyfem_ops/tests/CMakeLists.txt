if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

set(SRC_FILES
    test_polyfem_link.cpp
    test_python_repr.cpp
    test_interface_selection.cpp
    test_numpy_compat.cpp
    test_mesh_reduction.cpp
    test_polyfem_json.cpp
    test_polyfem_runner.cpp
    test_polyfem_in_process.cpp
    )

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
