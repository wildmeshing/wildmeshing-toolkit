set(SRC_FILES
    TriFaceSplit.hpp
    TriFaceSplit.cpp

    TriEdgeSwap.hpp
    TriEdgeSwap.cpp

    # TetSplit.hpp
    # TetSplit.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
