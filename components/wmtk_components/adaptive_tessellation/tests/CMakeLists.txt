
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes/tests")
include(catch2)
include(wmtk_data)
set(TEST_SOURCES
    test_operations.cpp
    test_sampling.cpp
    test_quadrature.cpp
    test_multimesh.cpp
    utils/unit_square_test_case.hpp
)
add_executable(AT_tests ${TEST_SOURCES})

target_link_libraries(AT_tests PUBLIC
    wmtk::toolkit
    wmtk::data
    Catch2::Catch2WithMain
    wmtk::test_tools
    adaptive_tessellation
)

FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
wmtk_copy_dll(AT_tests)

# Register unit tests
catch_discover_tests(AT_tests)