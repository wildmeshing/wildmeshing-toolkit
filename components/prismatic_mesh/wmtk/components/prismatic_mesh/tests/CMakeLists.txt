if(NOT BUILD_TESTING)
    return()
endif()

if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE
    test_prismatic_mesh.cpp
    test_read_msh.cpp
    test_target_positions.cpp
    test_optimization.cpp
    test_background_remeshing.cpp
    test_unlock.cpp
    test_prism_dominant.cpp
)
