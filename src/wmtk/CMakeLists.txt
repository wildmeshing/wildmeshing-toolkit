
set(SRC_FILES
    Accessor.cpp
    Accessor.hpp
    AttributeHandle.hpp
    Mesh.cpp
    Mesh.hpp
    MeshAttributes.cpp
    MeshAttributes.hpp
    Primitive.hpp
    Tuple.h
    utils/trimesh_topology_initialization.h
    utils/trimesh_topology_initialization.cpp
)
message(DEBUG "SRC_FILES: ${SRC_FILES}")
target_sources(wildmeshing_toolkit PRIVATE ${SOURCES})
