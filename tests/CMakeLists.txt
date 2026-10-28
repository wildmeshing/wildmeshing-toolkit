# Include Catch2 stuff
include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

# Sources
set(TEST_SOURCES
    2d_tests.cpp
    test_delaunay.cpp
    test_embed_triangles.cpp
    test_envelope.cpp
    test_geom.cpp
    test_io.cpp
    test_mesh_data.cpp
    test_knn.cpp
    test_link_condition.cpp
    test_nonmanifold.cpp
    test_operations.cpp
    test_predicates.cpp
    test_raw_simplex_collection.cpp
    test_read_triangle_mesh.cpp
    test_resolve_path.cpp
    test_winding_number.cpp
    test_simplex_navigation.cpp
    test_tuple.cpp
    test_orient.cpp
    test_orient_by_majority.cpp
    test_optimization.cpp
    test_threading.cpp
    test_vertex_coloring.cpp
    test_ring_lock.cpp
    test_slot_allocation.cpp
    test_screening.cpp
)

add_executable(wmtk_tests ${TEST_SOURCES})

# Macros
target_compile_definitions(wmtk_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

# Dependencies
include(wmtk_data)
target_link_libraries(wmtk_tests PUBLIC
    wmtk::toolkit
    wmtk::data
    Catch2::Catch2WithMain
)
add_sanitizers(wmtk_tests) # for detecting memory errors in unit tests
wmtk_copy_dll(wmtk_tests)

# Register unit tests
catch_discover_tests(wmtk_tests)

if(WMTK_BUILD_INTEGRATION_TESTS)
    message(STATUS "Include integration tests.")
    add_executable(wmtk_integration_tests integration_tests.cpp)

    # Macros
    target_compile_definitions(wmtk_integration_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

    # Dependencies
    include(wmtk_data)
    target_link_libraries(wmtk_integration_tests PUBLIC
        wmtk::toolkit
        wmtk::data
        Catch2::Catch2WithMain
    )
    add_sanitizers(wmtk_integration_tests) # for detecting memory errors in integration tests
    wmtk_copy_dll(wmtk_integration_tests)

    # Register unit tests.
    #
    # Well past ctest's 1500s default: the coarsening pass runs at the end of every tetwild and
    # triwild config and roughly triples what those cost, which is the point of it -- it trades
    # time for elements. Set on discovery rather than with set_tests_properties, because
    # catch_discover_tests registers the test at build time, when a configure-time property
    # call has nothing to attach to.
    catch_discover_tests(wmtk_integration_tests PROPERTIES TIMEOUT 7200)

endif()
# The NumPy memory-interface tests of the Python module, registered when the interpreter the
# module was built for has NumPy (the Linux CI jobs install it). The wheel builds run the same
# file through cibuildwheel.
if(TARGET wildmeshing AND Python_EXECUTABLE)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "import numpy"
        RESULT_VARIABLE WMTK_PYTHON_NUMPY_RESULT
        OUTPUT_QUIET ERROR_QUIET)
    if(WMTK_PYTHON_NUMPY_RESULT EQUAL 0)
        add_test(
            NAME python_memory_io
            COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=$<TARGET_FILE_DIR:wildmeshing>/.."
                    "${Python_EXECUTABLE}" -m unittest discover
                    -s "${PROJECT_SOURCE_DIR}/tests/python" -p "test_memory_io.py")
    else()
        message(STATUS "NumPy not found for ${Python_EXECUTABLE}: python_memory_io is not registered")
    endif()
endif()
