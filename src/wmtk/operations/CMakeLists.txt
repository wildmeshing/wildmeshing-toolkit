set(SRC_FILES
    Operation.hpp
    Operation.cpp
    TriMeshSplitEdgeOperation.cpp
    TriMeshSplitEdgeOperation.hpp
    TriMeshCollapseEdgeOperation.cpp
    TriMeshCollapseEdgeOperation.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
