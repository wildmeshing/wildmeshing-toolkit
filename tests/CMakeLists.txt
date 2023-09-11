# Include Catch2 stuff
include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
include(Catch)
include(wmtk_data)

lagrange_include_modules(io)

# Sources
set(TEST_SOURCES
	test_topology.cpp
    test_tuple.cpp
    test_tuple_2d.cpp
	test_tuple_3d.cpp
    test_io.cpp
    test_execution.cpp
    test_simplicial_complex.cpp
    test_2d_operations.cpp
    test_2d_operation_construction.cpp
    test_accessor.cpp
    test_example_meshes.cpp
    test_multi_mesh.cpp
    tools/DEBUG_PointMesh.hpp
    tools/DEBUG_TriMesh.hpp
    tools/DEBUG_TriMesh.cpp
    tools/TriMesh_examples.hpp
    tools/TriMesh_examples.cpp
    tools/redirect_logger_to_cout.hpp
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

add_subdirectory(components)
source_group("components" REGULAR_EXPRESSION "components\/.*\.(cpp|h|hpp)?$")

wmtk_copy_dll(wmtk_tests)

# Register unit tests
catch_discover_tests(wmtk_tests)
