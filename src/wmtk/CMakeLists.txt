
set(SRC_FILES
    Accessor.hpp
    Mesh.cpp
    Mesh.hpp
    PointMesh.cpp
    PointMesh.hpp
    EdgeMesh.hpp
    EdgeMesh.cpp
    TriMesh.cpp
    TriMesh.hpp
    TetMesh.cpp
    TetMesh.hpp
    EdgeMeshOperationExecutor.hpp
    EdgeMeshOperationExecutor.cpp
    TriMeshOperationExecutor.hpp
    TriMeshOperationExecutor.cpp
    TetMeshOperationExecutor.hpp
    TetMeshOperationExecutor.cpp
    Primitive.hpp
    Primitive.cpp
    Tuple.cpp
    Tuple.hpp
    Types.hpp
	Simplex.hpp
	SimplicialComplex.hpp
	SimplicialComplex.cpp
    Scheduler.hpp
    Scheduler.cpp
    MultiMeshManager.hpp
    MultiMeshManager.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(io)
add_subdirectory(utils)
add_subdirectory(attribute)
add_subdirectory(simplex)
add_subdirectory(operations)
add_subdirectory(autogen)
add_subdirectory(invariants)
add_subdirectory(multimesh)

