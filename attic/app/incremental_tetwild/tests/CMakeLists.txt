include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(incremental_tetwild_tests ${TEST_SOURCES})
target_link_libraries(incremental_tetwild_tests PUBLIC
    wmtk::incremental_tetwild
    Catch2::Catch2WithMain
)
wmtk_copy_dll(incremental_tetwild_tests)

set_target_properties(incremental_tetwild_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

catch_discover_tests(incremental_tetwild_tests)
