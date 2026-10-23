if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()
# The tests remesh with isotropic_remeshing, added before this component.
if(NOT TARGET wmtk::isotropic_remeshing)
    return()
endif()

add_component_test(${COMPONENT_NAME})

set(SRC_FILES
    test_surface_map.cpp
    test_curve_map.cpp
    test_tracking.cpp
)

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_test_${COMPONENT_NAME} PRIVATE wmtk::isotropic_remeshing)
