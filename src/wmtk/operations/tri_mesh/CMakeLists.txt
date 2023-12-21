set(SRC_FILES
    TriMeshOperation.hpp
    TriMeshOperation.cpp
    FaceSplit.hpp
    FaceSplit.cpp
    FaceSplitWithTag.hpp
    FaceSplitWithTag.cpp
    EdgeSplit.hpp
    EdgeSplit.cpp
    EdgeSplitAtMidpoint.hpp
    EdgeSplitAtMidpoint.cpp
    EdgeCollapse.hpp
    EdgeCollapse.cpp
    EdgeCollapseToMidpoint.hpp
    EdgeCollapseToMidpoint.cpp
    EdgeSwapBase.hpp
    EdgeSwapBase.cpp
    EdgeSwapValence.hpp
    EdgeSwapValence.cpp
    EdgeSplitWithTag.hpp
    EdgeSplitWithTag.cpp
    VertexLaplacianSmooth.hpp
    VertexLaplacianSmooth.cpp
    VertexTangentialLaplacianSmooth.hpp
    VertexTangentialLaplacianSmooth.cpp
    FaceSplitAtMidPoint.hpp
    FaceSplitAtMidPoint.cpp


    EdgeOperationData.hpp
    EdgeOperationData.cpp

    CollapseNewAttributeStrategy.hpp
    CollapseNewAttributeStrategy.cpp

    SplitNewAttributeStrategy.hpp
    SplitNewAttributeStrategy.cpp

    BasicSplitNewAttributeStrategy.hpp
    BasicSplitNewAttributeStrategy.cpp

    BasicCollapseNewAttributeStrategy.hpp
    BasicCollapseNewAttributeStrategy.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
