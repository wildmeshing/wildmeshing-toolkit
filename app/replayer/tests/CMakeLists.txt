include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
include(Catch)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(replayer_tests ${TEST_SOURCES})
target_link_libraries(replayer_tests PUBLIC
    wmtk::replayer_lib
    Catch2::Catch2
)
wmtk_copy_dll(replayer_tests)

set_target_properties(replayer_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

catch_discover_tests(replayer_tests)
