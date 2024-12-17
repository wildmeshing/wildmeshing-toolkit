set(SRC_FILES
    TriFaceSplit.hpp
    TriFaceSplit.cpp

    TriEdgeSwap.hpp
    TriEdgeSwap.cpp

    TetCellSplit.hpp
    TetCellSplit.cpp

    TetEdgeSwap.hpp
    TetEdgeSwap.cpp

    TetFaceSwap.hpp
    TetFaceSwap.cpp


    ProjectOperation.cpp
    ProjectOperation.hpp

    TriFaceSplit_alt.hpp
    TriFaceSplit_alt.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
