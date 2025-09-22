# bijective_map_app/CMakeLists.txt

cmake_minimum_required(VERSION 3.24.0)

# Configure options for InteractiveAndRobustMeshBooleans
set(ENABLE_SSE2 True CACHE BOOL "Enable SSE2 optimizations")
set(ENABLE_AVX2 True CACHE BOOL "Enable AVX2 optimizations")

# Add the existing InteractiveAndRobustMeshBooleans subdirectory
add_subdirectory(InteractiveAndRobustMeshBooleans)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(BijectiveMapApp)

if(USE_IGL_VIEWER)
    igl_include(glfw)
    igl_include(opengl)
    igl_include(stb)
    igl_include(imgui)
endif()

add_executable(bijective_map_app bijective_map_app.cpp track_operations.hpp track_operations.cpp track_operations_curve.hpp track_operations_curve.cpp handle_one_segment.cpp handle_collapse_curve.cpp handle_non_collapse_curve.cpp render_utils.hpp generate_iso_line.hpp track_point_app.cpp track_line_app.cpp vtu_utils.cpp generate_plane_curves.hpp generate_plane_curves.cpp batch_operation_log_reader.hpp batch_operation_log_reader.cpp handle_non_collapse.cpp handle_collapse.cpp get_curve_intersections.cpp)

target_link_libraries(bijective_map_app
    wmtk::toolkit
    wmtk::data

    # wmtk::components
    CLI11::CLI11
    igl::core
)

if(USE_IGL_VIEWER)
    target_link_libraries(bijective_map_app
        igl::opengl igl::glfw igl::stb igl::imgui
    )
endif()

target_compile_features(bijective_map_app PUBLIC cxx_std_17)

add_executable(bijective_map_app_tet bijective_map_app_tet.cpp track_operations_tet.hpp track_operations_tet.cpp FindPointTetMesh.cpp vtu_utils.cpp batch_operation_log_reader.hpp batch_operation_log_reader.cpp)
target_include_directories(bijective_map_app_tet PUBLIC
    ./InteractiveAndRobustMeshBooleans
    InteractiveAndRobustMeshBooleans/code/
    InteractiveAndRobustMeshBooleans/arrangements/code/
    InteractiveAndRobustMeshBooleans/arrangements/external/Indirect_Predicates/include/
)

# find_package(cinolib REQUIRED)
target_link_libraries(bijective_map_app_tet
    wmtk::toolkit
    wmtk::data

    # wmtk::components
    CLI11::CLI11
    igl::core
    nlohmann_json::nlohmann_json
    cinolib
    TBB::tbb
)

if(USE_IGL_VIEWER)
    target_link_libraries(bijective_map_app_tet
        igl::opengl igl::glfw igl::stb igl::imgui
    )
endif()

target_include_directories(bijective_map_app_tet PUBLIC ${PROJECT_SOURCE_DIR}/InteractiveAndRobustMeshBooleans/arrangements/external/abseil-cpp/)
target_compile_features(bijective_map_app_tet PUBLIC cxx_std_17)

# add_executable(test_find_point_tet test_findpointtetmesh.cpp FindPointTetMesh.cpp)
# target_link_libraries(test_find_point_tet
# wmtk::toolkit
# igl::core
# )
