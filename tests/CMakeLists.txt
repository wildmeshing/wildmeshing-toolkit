# Include Catch2 stuff
include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
include(Catch)

# Sources
set(TEST_SOURCES
    main.cpp
    test_topology.cpp)
add_executable(wmtk_tests ${TEST_SOURCES})

# Macros
target_compile_definitions(wmtk_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

# Dependencies
include(finite-diff)
target_link_libraries(wmtk_tests PUBLIC finitediff::finitediff)

include(wmtk_data)
lagrange_include_modules(io)
target_link_libraries(wmtk_tests PUBLIC
    wmtk::toolkit
    wmtk::data
    Catch2::Catch2
    lagrange::io
)
wmtk_copy_dll(wmtk_tests)

# Register unit tests
catch_discover_tests(wmtk_tests)
