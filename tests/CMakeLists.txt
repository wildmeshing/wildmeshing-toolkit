lagrange_include_modules(io)

# Sources
set(TEST_SOURCES
	test_topology.cpp
    test_tuple.cpp
    test_tuple_2d.cpp
	test_tuple_3d.cpp
    test_io.cpp
    test_simplicial_complex.cpp
)
add_executable(wmtk_tests ${TEST_SOURCES})


# Dependencies
include(finite-diff)
target_link_libraries(wmtk_tests PUBLIC finitediff::finitediff)

target_link_libraries(wmtk_tests PUBLIC
    wmtk::toolkit
    wmtk::data
    Catch2::Catch2WithMain
    lagrange::io
)
wmtk_copy_dll(wmtk_tests)

# Register unit tests
catch_discover_tests(wmtk_tests)
