# Sources
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/tests")

include(catch2)
include(wmtk_data)


add_executable(${WMTK_COMPONENT_PREFIX}_tests ${WMTKC_TEST_SOURCES})


target_link_libraries(${WMTK_COMPONENT_PREFIX}_tests PUBLIC
wmtk_test_tools
wmtk::components)

FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)


wmtk_copy_dll(${WMTK_COMPONENT_PREFIX}_tests)

# Register unit tests
catch_discover_tests(${WMTK_COMPONENT_PREFIX}_tests)
