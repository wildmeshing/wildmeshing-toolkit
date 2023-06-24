
set(SRC_FILES
    Accessor.cpp
    Accessor.hpp
    AttributeHandle.hpp
    Mesh.cpp
    Mesh.hpp
    MeshAttributes.cpp
    MeshAttributes.hpp
    TriMesh.cpp
    TriMesh.hpp
    Primitive.hpp
    Tuple.hpp
    Types.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(io)
add_subdirectory(utils)
add_subdirectory(autogen)
