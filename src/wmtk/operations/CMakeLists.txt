set(SRC_FILES
    Operation.hpp
    Operation.cpp
    OperationFactory.hpp
    OperationFactory.cpp
    TriMeshSplitEdgeOperation.cpp
    TriMeshSplitEdgeOperation.hpp
    TriMeshCollapseEdgeOperation.cpp
    TriMeshCollapseEdgeOperation.hpp
    TriMeshSwapEdgeOperation.cpp
    TriMeshSwapEdgeOperation.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
