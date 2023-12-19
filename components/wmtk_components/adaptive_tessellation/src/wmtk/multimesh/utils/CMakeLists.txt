
set(SRC_FILES
    map_sibling_edge_meshes.hpp
    map_sibling_edge_meshes.cpp
    find_critical_points.hpp
    find_critical_points.cpp
    )
target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})