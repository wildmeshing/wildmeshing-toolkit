set(SRC_FILES
    UpdateEdgeOperationMultiMeshMapFunctor.cpp
    UpdateEdgeOperationMultiMeshMapFunctor.hpp
    #multi_mesh_edge_split.cpp
    multi_mesh_edge_split.hpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

