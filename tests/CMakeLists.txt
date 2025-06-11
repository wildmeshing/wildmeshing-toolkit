# Include Catch2 stuff
include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

# Sources
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
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
