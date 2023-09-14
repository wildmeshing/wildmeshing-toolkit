
set(SRC_FILES
    Logger.cpp
    Logger.hpp
    trimesh_topology_initialization.h
    trimesh_topology_initialization.cpp
    tetmesh_topology_initialization.h
    tetmesh_topology_initialization.cpp
    getRSS.cpp
    getRSS.h
    Rational.hpp
    mesh_utils.hpp
    mesh_utils.cpp
    triangle_helper_functions.hpp
    triangle_helper_functions.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
