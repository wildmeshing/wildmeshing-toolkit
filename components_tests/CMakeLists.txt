# Sources
set(TEST_SOURCES
	test_input.cpp
)
add_executable(wmtk_components_tests ${TEST_SOURCES})

target_link_libraries(wmtk_components_tests PUBLIC
    wildmeshing_components
    wmtk::data
    Catch2::Catch2WithMain
)
wmtk_copy_dll(wmtk_components_tests)

# Register unit tests
catch_discover_tests(wmtk_components_tests)
