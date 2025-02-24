# Include Catch2 stuff

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/tests")

include(catch2)
include(wmtk_data)

set(TEST_TOOLS
tools/all_valid_local_tuples.hpp
tools/all_valid_local_tuples.cpp
tools/is_free.hpp
tools/is_free.cpp
tools/add_free_child_mesh.hpp
tools/add_free_child_mesh.cpp
tools/DEBUG_PointMesh.hpp
tools/DEBUG_EdgeMesh.hpp
tools/DEBUG_EdgeMesh.cpp
tools/DEBUG_TriMesh.hpp
tools/DEBUG_TriMesh.cpp
tools/EdgeMesh_examples.hpp
tools/EdgeMesh_examples.cpp
tools/TriMesh_examples.hpp
tools/TriMesh_examples.cpp
tools/redirect_logger_to_cout.hpp
tools/TetMesh_examples.hpp
tools/TetMesh_examples.cpp
tools/DEBUG_TetMesh.hpp
tools/DEBUG_TetMesh.cpp
tools/single_simplex_mesh.hpp
tools/single_simplex_mesh.cpp
tools/global_index_max_subdart_size.hpp
tools/global_index_max_subdart_size.cpp
tools/global_ids.hpp
tools/global_ids.cpp
tools/TestTools.hpp
tools/TestTools.cpp

tools/DEBUG_Mesh.hpp
tools/DEBUG_Mesh.cpp
tools/DEBUG_MultiMeshManager.hpp
tools/DEBUG_MultiMeshManager.cpp

)
add_library(wmtk_test_tools ${TEST_TOOLS})
target_include_directories(wmtk_test_tools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
## Dependencies
include(finite-diff)

target_link_libraries(wmtk_test_tools PUBLIC
    wmtk::toolkit
    wmtk::data
    Catch2::Catch2
    finitediff::finitediff
)

# Sources
set(TEST_SOURCES
    test_topology.cpp
    test_mesh.cpp
    test_tuple.cpp
    test_tuple_1d.cpp
    test_tuple_2d.cpp
	test_tuple_3d.cpp
    test_execution.cpp
    test_2d_invariants.cpp
    test_3d_invariants.cpp
    test_simplex_collection.cpp
    test_simplicial_complex.cpp
    test_1d_operations.cpp
    test_example_meshes.cpp
    test_3d_operations.cpp
    test_mesh_variant.cpp
    test_variant_metaprogramming.cpp
    test_tuple_metaprogramming.cpp
    test_autodiff.cpp
    test_local_switch_search.cpp
    test_primitive.cpp
    test_performance.cpp
)
add_executable(wmtk_tests ${TEST_SOURCES})
target_link_libraries(wmtk_tests PUBLIC wmtk_test_tools
    Catch2::Catch2WithMain
    wmtk::toolkit
    igl::core
    )

FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

function(add_subdirectory_with_source_group)
    set(SUBDIR_NAME "${ARGV0}")
    add_subdirectory(${SUBDIR_NAME})
    source_group("${SUBDIR_NAME}" REGULAR_EXPRESSION "${SUBDIR_NAME}\/.*\.(cpp|h|hpp)?$")
endfunction()

add_subdirectory_with_source_group(function)
add_subdirectory_with_source_group(simplex)
add_subdirectory_with_source_group(io)
add_subdirectory_with_source_group(utils)
add_subdirectory_with_source_group(multimesh)
add_subdirectory_with_source_group(operations)
add_subdirectory_with_source_group(attributes)
add_subdirectory_with_source_group(autogen)
add_subdirectory_with_source_group(submesh)
add_subdirectory_with_source_group(invariants)




wmtk_copy_dll(wmtk_tests)

# Register unit tests
catch_discover_tests(wmtk_tests)
