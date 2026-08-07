if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})
target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE test_prismatic_mesh.cpp)

add_executable(wmtk_prismatic_regression_compare regression_compare.cpp)
target_link_libraries(
    wmtk_prismatic_regression_compare
    PRIVATE wmtk::prismatic_mesh wmtk::toolkit)
set_target_properties(
    wmtk_prismatic_regression_compare
    PROPERTIES FOLDER wmtk_components_tests)
