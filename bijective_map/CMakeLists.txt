# bijective_map_app/CMakeLists.txt

cmake_minimum_required(VERSION 3.24.0)
project(BijectiveMapApp)

igl_include(glfw)
igl_include(opengl)
igl_include(stb)

add_executable(bijective_map_app bijective_map_app.cpp track_operations.hpp track_operations.cpp render_utils.hpp)

target_link_libraries(bijective_map_app
    wmtk::toolkit
    wmtk::data
    wmtk::components
    CLI11::CLI11
    igl::core igl::opengl igl::glfw igl::stb
)

target_compile_features(bijective_map_app PUBLIC cxx_std_17)
