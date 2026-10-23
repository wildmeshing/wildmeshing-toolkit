if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

set(SRC_FILES
    test_qslim.cpp
    test_qslim_boundary.cpp
)

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

# The tracking example rebuilds the map with surface_map. It is added after this component, so its
# option may not be set yet; unset, it defaults to on.
if(NOT DEFINED WMTK_ENABLE_COMPONENT_surface_map OR WMTK_ENABLE_COMPONENT_surface_map)
    target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE test_qslim_tracking.cpp)
    target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk::surface_map)
endif()
