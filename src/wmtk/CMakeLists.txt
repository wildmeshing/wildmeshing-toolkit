
set(SRC_FILES
    Accessor.hpp
    Mesh.cpp
    Mesh.hpp
    PointMesh.cpp
    PointMesh.hpp
    TriMesh.cpp
    TriMesh.hpp
    TetMesh.cpp
    TetMesh.hpp
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



include(stb)
include(tinyexr)
target_link_libraries(wildmeshing_toolkit PUBLIC
    miniz # MTAO: I had a build issue with windows not finding miniz at linktime - adding here to make sure it's there?
    tinyexr
    stb::image)

add_subdirectory(io)
add_subdirectory(utils)
add_subdirectory(attribute)
add_subdirectory(simplex)
add_subdirectory(operations)
add_subdirectory(autogen)

add_subdirectory(invariants)

add_subdirectory(function)
add_subdirectory(image)

include(stb)
include(tinyexr)
target_link_libraries(wildmeshing_toolkit PUBLIC
    miniz # MTAO: I had a build issue with windows not finding miniz at linktime - adding here to make sure it's there?
    tinyexr
    stb::image)
