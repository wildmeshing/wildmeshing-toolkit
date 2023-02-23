# Include Catch2 stuff
include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
include(Catch)

# Sources
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(test_wmtk ${TEST_SOURCES})

# Macros
target_compile_definitions(test_wmtk PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

# Dependencies
include(wmtk_data)
target_link_libraries(test_wmtk PUBLIC
    wmtk::toolkit
    wmtk::data
    Catch2::Catch2
)
wmtk_copy_dll(test_wmtk)

# Register unit tests
catch_discover_tests(test_wmtk)
