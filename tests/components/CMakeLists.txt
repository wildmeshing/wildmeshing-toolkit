# Sources
set(TEST_SOURCES
	test_component_input.cpp
    test_component_mesh_info.cpp
    test_component_output.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})

target_link_libraries(wmtk_tests PUBLIC
    wildmeshing_components
)