set(SRC_FILES
    VertexLaplacianSmooth.hpp
    VertexLaplacianSmooth.cpp

    VertexTangentialLaplacianSmooth.hpp
    VertexTangentialLaplacianSmooth.cpp

    RoundRationalToDouble.hpp
    RoundRationalToDouble.cpp

    ExtractOperationTuplesFunctor.hpp
    EdgeCollapseFunctor.cpp
    EdgeCollapseFunctor.hpp
    EdgeSplitFunctor.cpp
    EdgeSplitFunctor.hpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
if(WMTK_ENABLE_MULTIMESH)
set(SRC_FILES
    UpdateEdgeOperationMultiMeshMapFunctor.cpp
    UpdateEdgeOperationMultiMeshMapFunctor.hpp
    multi_mesh_edge_collapse.cpp
    multi_mesh_edge_collapse.hpp
    multi_mesh_edge_split.cpp
    multi_mesh_edge_split.hpp
    UpdateVertexMultiMeshMapHash.hpp
    UpdateVertexMultiMeshMapHash.cpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
endif()

