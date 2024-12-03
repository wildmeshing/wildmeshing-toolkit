# bijective_map_app/CMakeLists.txt

cmake_minimum_required(VERSION 3.24.0)
project(BijectiveMapApp)

igl_include(glfw)
igl_include(opengl)
igl_include(stb)
igl_include(imgui)
add_executable(bijective_map_app bijective_map_app.cpp track_operations.hpp track_operations.cpp render_utils.hpp generate_iso_line.hpp track_point_app.cpp track_line_app.cpp)

target_link_libraries(bijective_map_app
    wmtk::toolkit
    wmtk::data

    # wmtk::components
    CLI11::CLI11
    igl::core igl::opengl igl::glfw igl::stb
)
target_compile_features(bijective_map_app PUBLIC cxx_std_17)

add_executable(bijective_map_app_tet bijective_map_app_tet.cpp track_operations_tet.hpp track_operations_tet.cpp FindPointTetMesh.cpp)
target_link_libraries(bijective_map_app_tet
    wmtk::toolkit
    wmtk::data

    # wmtk::components
    CLI11::CLI11
    igl::core igl::opengl igl::glfw igl::stb
)
target_compile_features(bijective_map_app_tet PUBLIC cxx_std_17)

add_executable(test_find_point_tet test_findpointtetmesh.cpp FindPointTetMesh.cpp)
target_link_libraries(test_find_point_tet
    igl::core
)
