set(SRC_FILES
    Operation.hpp
    Operation.cpp
    OperationFactory.hpp
    OperationFactory.cpp
    TriMeshSplitEdgeOperation.hpp
    TriMeshSplitEdgeOperation.cpp
    TriMeshCollapseEdgeOperation.hpp
    TriMeshCollapseEdgeOperation.cpp
    TriMeshSwapEdgeOperation.hpp
    TriMeshSwapEdgeOperation.cpp
    TriMeshVertexSmoothOperation.hpp
    TriMeshVertexSmoothOperation.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
