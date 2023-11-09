# ###############################################################################
# WMTK JSON APP
# ###############################################################################

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/components")

add_executable(wmtk_app main.cpp)
include(jse)
include(cli11)

target_compile_options(wmtk_app PRIVATE "-rdynamic")
target_link_libraries(wmtk_app PUBLIC
    wmtk::components
    CLI11::CLI11
    jse::jse
)

target_compile_definitions(wmtk_app PUBLIC WMTK_APP_INPUT_SPEC="${CMAKE_SOURCE_DIR}/wmtk_spec.json")
