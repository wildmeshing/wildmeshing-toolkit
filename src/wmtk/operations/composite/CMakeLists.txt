set(SRC_FILES
    # FaceSplit.hpp
    # FaceSplit.cpp

    EdgeSwap.hpp
    EdgeSwap.cpp

    # TetSplit.hpp
    # TetSplit.cpp

    # VertexLaplacianSmooth.hpp
    # VertexLaplacianSmooth.cpp
    # VertexTangentialLaplacianSmooth.hpp
    # VertexTangentialLaplacianSmooth.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
