set(SRC_FILES
    AttributesUpdateBase.hpp
    AttributesUpdateBase.cpp

    Operation.hpp
    Operation.cpp

    MeshOperation.hpp
    MeshOperation.cpp

    EdgeCollapse.hpp
    EdgeCollapse.cpp

    EdgeSplit.hpp
    EdgeSplit.cpp

    NewAttributeStrategy.hpp
    NewAttributeStrategy.cpp
    CollapseNewAttributeStrategy.hpp
    CollapseNewAttributeStrategy.cpp
    SplitNewAttributeStrategy.hpp
    SplitNewAttributeStrategy.cpp

    EdgeOperationData.hpp
    EdgeOperationData.cpp

    OptimizationSmoothing.cpp
    OptimizationSmoothing.hpp

    VertexLaplacianSmooth.hpp
    VertexLaplacianSmooth.cpp

    AttributeTransferStrategyBase.hpp
    AttributeTransferStrategy.hpp
    AttributeTransferStrategy.cpp
    AttributeTransferStrategyBase.cpp

    # VertexTangentialLaplacianSmooth.hpp
    # VertexTangentialLaplacianSmooth.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(edge_mesh)
add_subdirectory(tri_mesh)
#TODODIscuss
# add_subdirectory(tet_mesh)

add_subdirectory(composite)

add_subdirectory(utils)

