# Include Catch2 stuff
include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

# Sources
set(TEST_SOURCES
    test_delaunay.cpp
    test_geom.cpp
    test_io.cpp
    test_knn.cpp
    test_link_condition.cpp
    test_manifold.cpp
    test_operations.cpp
    test_raw_simplex_collection.cpp
    test_simplex_navigation.cpp
    test_tuple.cpp
    test_orient.cpp
    test_optimization.cpp
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
    wmtk_copy_dll(wmtk_integration_tests)

    # Register unit tests
    catch_discover_tests(wmtk_integration_tests)

endif()