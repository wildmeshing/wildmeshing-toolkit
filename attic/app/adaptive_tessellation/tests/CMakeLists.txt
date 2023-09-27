include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
include(Catch)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(adaptive_tessellation_tests ${TEST_SOURCES})
target_link_libraries(adaptive_tessellation_tests PUBLIC
    wmtk::adaptive_tessellation
    Catch2::Catch2
)
wmtk_copy_dll(adaptive_tessellation_tests)

set_target_properties(adaptive_tessellation_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

catch_discover_tests(adaptive_tessellation_tests)
