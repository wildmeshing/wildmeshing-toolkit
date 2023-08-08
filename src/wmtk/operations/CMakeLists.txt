set(SRC_FILES
    Operation.hpp
    Operation.cpp
    OperationFactory.hpp
    OperationFactory.cpp
    OperationQueue.hpp
    TriMeshSplitEdgeOperation.hpp
    TriMeshSplitEdgeOperation.cpp
    TriMeshCollapseEdgeOperation.hpp
    TriMeshCollapseEdgeOperation.cpp
    TriMeshSwapEdgeOperation.hpp
    TriMeshSwapEdgeOperation.cpp
    TriMeshVertexSmoothOperation.hpp
    TriMeshVertexSmoothOperation.cpp
	TriMeshVertexSmoothOperationFactory.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
