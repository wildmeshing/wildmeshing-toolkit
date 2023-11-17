set(SRC_FILES
    TetMeshOperation.hpp
    TetMeshOperation.cpp
    EdgeSplit.hpp
    EdgeSplit.cpp
    EdgeCollapse.hpp
    EdgeCollapse.cpp
    TetSplit.hpp
    TetSplit.cpp
    TetSplitWithTags.hpp
    TetSplitWithTags.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
