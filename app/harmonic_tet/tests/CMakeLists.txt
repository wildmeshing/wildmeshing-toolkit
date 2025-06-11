include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(harmonic_tet_tests ${TEST_SOURCES})
target_link_libraries(harmonic_tet_tests PUBLIC
    wmtk::harmonic_tet
    Catch2::Catch2WithMain
)
wmtk_copy_dll(harmonic_tet_tests)

set_target_properties(harmonic_tet_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

catch_discover_tests(harmonic_tet_tests)

