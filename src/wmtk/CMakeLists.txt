
set(SRC_FILES
    Accessor.cpp
    Accessor.hpp
    AccessorBase.cpp
    AccessorBase.hpp
    AttributeHandle.hpp
    Mesh.cpp
    Mesh.hpp
    Attribute.hpp
    Attribute.cpp
    MeshAttributes.cpp
    MeshAttributes.hpp
    PointMesh.cpp
    PointMesh.hpp
    TriMesh.cpp
    TriMesh.hpp
    TetMesh.cpp
    TetMesh.hpp
    TriMeshOperation.hpp
    TriMeshOperation.cpp
    Primitive.hpp
    Tuple.cpp
    Tuple.hpp
    Types.hpp
	Simplex.hpp
	SimplicialComplex.hpp
	SimplicialComplex.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(io)
add_subdirectory(utils)
add_subdirectory(autogen)
