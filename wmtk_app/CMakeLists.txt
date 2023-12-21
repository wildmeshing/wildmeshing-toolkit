# ###############################################################################
# WMTK JSON APP
# ###############################################################################

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/components")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/tests")

add_executable(wmtk_app main.cpp)
add_executable(extremeopt_app extremeopt_app.cpp)
include(jse)
include(cli11)
include(wmtk_data)

target_compile_options(wmtk_app PRIVATE "-rdynamic")
target_link_libraries(wmtk_app PUBLIC
    wmtk::components
    CLI11::CLI11
    jse::jse
)

target_compile_options(extremeopt_app PRIVATE "-rdynamic")
target_link_libraries(extremeopt_app PUBLIC
    wmtk::data
    wmtk::components
    CLI11::CLI11
    jse::jse
)

target_compile_definitions(wmtk_app PUBLIC WMTK_APP_INPUT_SPEC="${CMAKE_SOURCE_DIR}/wmtk_spec.json")
