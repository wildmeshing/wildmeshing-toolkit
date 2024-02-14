
set(SRC_FILES
    map_sibling_edge_meshes.hpp
    map_sibling_edge_meshes.cpp
    find_critical_points.hpp
    find_critical_points.cpp

    edge_meshes_parameterization.hpp
    edge_meshes_parameterization.cpp
    )
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
