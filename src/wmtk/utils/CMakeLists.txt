
set(SRC_FILES
    Cache.cpp
    Cache.hpp
    Logger.cpp
    Logger.hpp
    edgemesh_topology_initialization.h
    edgemesh_topology_initialization.cpp
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
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})


add_subdirectory(metaprogramming)
