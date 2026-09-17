if(NOT BUILD_TESTING)
    return()
endif()

if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE
    test_prismatic_mesh.cpp
)
