set(SRC_FILES
    EdgeOperationData.hpp

    TriMeshOperation.hpp
    TriMeshOperation.cpp

    # FaceSplit.hpp
    # FaceSplit.cpp

    # EdgeSplit.hpp
    # EdgeSplit.cpp

    # EdgeCollapse.hpp
    # EdgeCollapse.cpp

    # EdgeSwap.hpp
    # EdgeSwap.cpp

    # VertexLaplacianSmooth.hpp
    # VertexLaplacianSmooth.cpp
    # VertexTangentialLaplacianSmooth.hpp
    # VertexTangentialLaplacianSmooth.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
