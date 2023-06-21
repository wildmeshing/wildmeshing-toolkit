# ###############################################################################
# WMTK JSON APP
# ###############################################################################

add_executable(wmtk_json_app main.cpp)
target_compile_options(wmtk_json_app PRIVATE "-rdynamic")
target_link_libraries(wmtk_json_app PUBLIC wildmeshing_toolkit
CLI11::CLI11
)