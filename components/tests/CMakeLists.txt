# Sources
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/tests")

include(catch2)
include(wmtk_data)

set(WMTK_COMPONENT_TEST_TARGET ${WMTK_COMPONENT_PREFIX}_tests)

file(GLOB WMTKC_TEST_SOURCES test_*.cpp)
add_executable(${WMTK_COMPONENT_TEST_TARGET} ${WMTKC_TEST_SOURCES})
target_sources(${WMTK_COMPONENT_TEST_TARGET} PRIVATE integration_test.cpp)


target_link_libraries(${WMTK_COMPONENT_TEST_TARGET} PUBLIC
wmtk_test_tools
Catch2::Catch2WithMain
wmtk::components)
target_include_directories(${WMTK_COMPONENT_TEST_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_definitions(${WMTK_COMPONENT_TEST_TARGET} PRIVATE WMTK_TEST_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")

FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)


wmtk_copy_dll(${WMTK_COMPONENT_PREFIX}_tests)

# Register unit tests
catch_discover_tests(${WMTK_COMPONENT_PREFIX}_tests)
