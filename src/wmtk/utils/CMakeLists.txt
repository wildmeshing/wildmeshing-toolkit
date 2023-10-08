
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
    TupleInspector.hpp
    as_mesh_variant.hpp
    as_mesh_variant.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
