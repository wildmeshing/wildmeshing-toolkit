# Sources
#TODOfix: restore tests
set(TEST_SOURCES
    test_component_adaptive_tessellation.cpp
    test_component_delaunay.cpp
	test_component_input.cpp
    test_component_mesh_info.cpp
    test_component_output.cpp
	# test_component_isotropic_remeshing.cpp
    test_smoothing.cpp
    # test_component_regular_space.cpp
    # test_component_marching.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})

target_link_libraries(wmtk_tests PUBLIC
    wmtk::components
)
