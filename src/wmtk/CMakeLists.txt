
set(SRC_FILES
    Cell.cpp
    Cell.hpp
    Mesh.cpp
    Mesh_attributes.cpp #the attribute parts of Mesh class
    Mesh_construction.cpp #the construction/destruction parts of Mesh class
    Mesh.hpp
    Mesh_hash.cpp
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
    PrimitiveType.hpp
    PrimitiveType.cpp
    Primitive.hpp
    Primitive.cpp
    Tuple.hxx
    Tuple.hpp
    Types.hpp
    Scheduler.hpp
    Scheduler.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(io)
add_subdirectory(utils)
add_subdirectory(attribute)
add_subdirectory(simplex)
add_subdirectory(operations)
add_subdirectory(autogen)
#
add_subdirectory(invariants)
if(WMTK_ENABLE_MULTIMESH)
    target_sources(wildmeshing_toolkit PRIVATE 
    Mesh_multimesh.cpp
        MultiMeshManager.hpp)
add_subdirectory(multimesh)
endif()
#
add_subdirectory(function)

