set(SRC_FILES
    MultiMeshEdgeCollapseFunctor.cpp
    MultiMeshEdgeCollapseFunctor.hpp
    MultiMeshEdgeSplitFunctor.cpp
    MultiMeshEdgeSplitFunctor.hpp
    UpdateEdgeOperationMultiMeshMapFunctor.cpp
    UpdateEdgeOperationMultiMeshMapFunctor.hpp
    multi_mesh_edge_collapse.cpp
    multi_mesh_edge_collapse.hpp
    multi_mesh_edge_split.cpp
    multi_mesh_edge_split.hpp
    UpdateVertexMultiMeshMapHash.hpp
    UpdateVertexMultiMeshMapHash.cpp

    get_local_trimesh.cpp

    VertexLaplacianSmooth.hpp
    VertexLaplacianSmooth.cpp

    VertexTangentialLaplacianSmooth.hpp
    VertexTangentialLaplacianSmooth.cpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

