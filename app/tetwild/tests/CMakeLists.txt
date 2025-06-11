include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(tetwild_tests ${TEST_SOURCES})
target_link_libraries(tetwild_tests PUBLIC
    wmtk::tetwild
    Catch2::Catch2WithMain
)
wmtk_copy_dll(tetwild_tests)

set_target_properties(tetwild_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

catch_discover_tests(tetwild_tests)
