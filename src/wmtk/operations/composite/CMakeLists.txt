set(SRC_FILES
    TriFaceSplit.hpp
    TriFaceSplit.cpp

    TriEdgeSwap.hpp
    TriEdgeSwap.cpp

    TetCellSplit.hpp
    TetCellSplit.cpp

    TetEdgeSwap.hpp
    TetEdgeSwap.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
