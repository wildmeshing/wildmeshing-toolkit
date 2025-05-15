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

    get_local_mesh.cpp

    local_joint_flatten.hpp
    local_joint_flatten.cpp

    VertexLaplacianSmooth.hpp
    VertexLaplacianSmooth.cpp

    VertexTangentialLaplacianSmooth.hpp
    VertexTangentialLaplacianSmooth.cpp

    TetMesh_Embedding.hpp
    TetMesh_Embedding.cpp

    to_three_connected.hpp
    to_three_connected.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
