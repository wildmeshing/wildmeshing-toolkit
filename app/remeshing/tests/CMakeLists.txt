include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(remeshing_tests ${TEST_SOURCES})
target_link_libraries(remeshing_tests PUBLIC
    wmtk::remeshing_lib
    Catch2::Catch2WithMain
)
wmtk_copy_dll(remeshing_tests)

set_target_properties(remeshing_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

catch_discover_tests(remeshing_tests)
