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

    # Register unit tests
    catch_discover_tests(wmtk_integration_tests)

    # Golden-hash integration check: run the blessed integration tests through
    # wmtk_app single-threaded and verify each run's output hashes against the
    # reference stored in data/integration_tests/integration_tests.json. Build with
    # -DWMTK_FP_STRICT=ON for the outputs to be bit-identical across OS/compiler.
    # See tests/integration_hashes.py.
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(
            NAME integration_hashes
            COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_CURRENT_SOURCE_DIR}/integration_hashes.py"
                --app "$<TARGET_FILE:wmtk_app>"
                check
        )
        set_tests_properties(integration_hashes PROPERTIES
            ENVIRONMENT "WMTK_DATA_DIR=${WMTK_DATA_ROOT}/integration_tests")
    else()
        message(WARNING "Python3 not found: integration_hashes ctest not registered.")
    endif()

endif()