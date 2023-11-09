set(SRC_FILES
    TetMeshOperation.hpp
    TetMeshOperation.cpp
    TetEdgeSplit.hpp
    TetEdgeSplit.cpp
    TetEdgeCollapse.hpp
    TetEdgeCollapse.cpp
    TetSplit.hpp
    TetSplit.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
