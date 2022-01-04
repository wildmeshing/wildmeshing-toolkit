include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
include(Catch)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(tests_edge_collapse ${TEST_SOURCES})
target_link_libraries(tests_edge_collapse PUBLIC
    wmtk::edge_collapse
    Catch2::Catch2
)

catch_discover_tests(tests_edge_collapse)
