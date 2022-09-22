include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
include(Catch)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(triwild_tests ${TEST_SOURCES})
target_link_libraries(triwild_tests PUBLIC
    wmtk::triwild
    Catch2::Catch2
)
wmtk_copy_dll(triwild_tests)

set_target_properties(triwild_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

catch_discover_tests(triwild_tests)
