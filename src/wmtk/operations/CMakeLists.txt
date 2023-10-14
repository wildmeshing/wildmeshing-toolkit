set(SRC_FILES
    Operation.hpp
    Operation.cpp
    TupleOperation.hpp
    TupleOperation.cpp
    OperationFactory.hpp
    OperationFactory.cpp
    OperationDifferentiableSmoothFactory.hpp
    OperationQueue.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(edge_mesh)
add_subdirectory(tri_mesh)
add_subdirectory(tet_mesh)