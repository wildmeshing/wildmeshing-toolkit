set(SRC_FILES
    TetMeshOperation.hpp
    TetMeshOperation.cpp
    TetEdgeSplit.hpp
    TetEdgeSplit.cpp
    TetEdgeSplitWithTags.hpp
    TetEdgeSplitWithTags.cpp
    TetEdgeCollapse.hpp
    TetEdgeCollapse.cpp
    TetSplit.hpp
    TetSplit.cpp
    TetSplitWithTags.hpp
    TetSplitWithTags.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
