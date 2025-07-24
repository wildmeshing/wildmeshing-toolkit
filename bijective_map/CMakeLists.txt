# bijective_map_app/CMakeLists.txt

cmake_minimum_required(VERSION 3.24.0)

# Enable FetchContent
include(FetchContent)

# Configure FetchContent for InteractiveAndRobustMeshBooleans
FetchContent_Declare(
    InteractiveAndRobustMeshBooleans
    GIT_REPOSITORY https://github.com/zlyfunction/InteractiveAndRobustMeshBooleans.git
    GIT_TAG for_wmtk
)

# Configure options for InteractiveAndRobustMeshBooleans
set(ENABLE_SSE2 True CACHE BOOL "Enable SSE2 optimizations")
set(ENABLE_AVX2 True CACHE BOOL "Enable AVX2 optimizations")

# Make the content available
FetchContent_MakeAvailable(InteractiveAndRobustMeshBooleans)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(BijectiveMapApp)

igl_include(glfw)
igl_include(opengl)
igl_include(stb)
igl_include(imgui)
add_executable(bijective_map_app bijective_map_app.cpp track_operations.hpp track_operations.cpp track_operations_curve.hpp track_operations_curve.cpp render_utils.hpp generate_iso_line.hpp track_point_app.cpp track_line_app.cpp vtu_utils.cpp)

target_link_libraries(bijective_map_app
    wmtk::toolkit
    wmtk::data

    # wmtk::components
    CLI11::CLI11
    igl::core igl::opengl igl::glfw igl::stb
)
target_compile_features(bijective_map_app PUBLIC cxx_std_17)

add_executable(bijective_map_app_tet bijective_map_app_tet.cpp track_operations_tet.hpp track_operations_tet.cpp FindPointTetMesh.cpp vtu_utils.cpp)
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
    igl::core igl::opengl igl::glfw igl::stb
    nlohmann_json::nlohmann_json
    cinolib
    TBB::tbb
)
target_include_directories(bijective_map_app_tet PUBLIC ${PROJECT_SOURCE_DIR}/InteractiveAndRobustMeshBooleans/arrangements/external/abseil-cpp/)
target_compile_features(bijective_map_app_tet PUBLIC cxx_std_17)

# add_executable(test_find_point_tet test_findpointtetmesh.cpp FindPointTetMesh.cpp)
# target_link_libraries(test_find_point_tet
# wmtk::toolkit
# igl::core
# )
