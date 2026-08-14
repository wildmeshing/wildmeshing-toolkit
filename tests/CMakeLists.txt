# Include Catch2 stuff
include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

# Sources
set(TEST_SOURCES
    2d_tests.cpp
    test_delaunay.cpp
    test_envelope.cpp
    test_geom.cpp
    test_io.cpp
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
    test_optimization.cpp
    test_threading.cpp
    test_ring_lock.cpp
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