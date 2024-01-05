# Sources
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/tests")

include(catch2)
include(wmtk_data)

set(TEST_SOURCES
    test_component_adaptive_tessellation.cpp
    test_component_delaunay.cpp
	test_component_input.cpp
    test_component_mesh_info.cpp
    test_component_output.cpp
	test_component_isotropic_remeshing.cpp
    test_component_regular_space.cpp
    test_component_tag_intersection.cpp
    test_component_marching.cpp
    test_wildmeshing.cpp
    test_component_multimesh.cpp
)


add_executable(${WMTK_COMPONENT_PREFIX}_tests ${TEST_SOURCES})

target_sources(${WMTK_COMPONENT_PREFIX}_tests PRIVATE ${TEST_SOURCES})

target_link_libraries(${WMTK_COMPONENT_PREFIX}_tests PUBLIC
wmtk_test_tools
wmtk::components)

FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)


wmtk_copy_dll(${WMTK_COMPONENT_PREFIX}_tests)

# Register unit tests
catch_discover_tests(${WMTK_COMPONENT_PREFIX}_tests)
