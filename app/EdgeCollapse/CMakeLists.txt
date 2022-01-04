# Main lib
add_library(wmtk_edge_collapse
    src/EdgeCollapse/EdgeCollapse.h
    src/EdgeCollapse/QEC.cpp
    src/EdgeCollapse/ShortestEdgeCollapse.cpp
)
add_library(wmtk::edge_collapse ALIAS wmtk_edge_collapse)

target_include_directories(wmtk_edge_collapse PUBLIC src)
target_link_libraries(wmtk_edge_collapse PUBLIC igl::predicates FastEnvelope app_shared)

# Executable
add_executable(edge_collapse app/main.cpp)
target_link_libraries(edge_collapse PUBLIC wmtk::edge_collapse)

set_target_properties(edge_collapse PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/app")

# Unit tests
add_subdirectory(tests)
