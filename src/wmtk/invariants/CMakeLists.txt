set(SRC_FILES
    Invariant.hpp
    Invariant.cpp

    EnvelopeInvariant.hpp
    EnvelopeInvariant.cpp

    FunctionInvariant.hpp
    FunctionInvariant.cpp
    InvariantCollection.hpp
    InvariantCollection.cpp
    InteriorSimplexInvariant.hpp
    InteriorSimplexInvariant.cpp
    InteriorVertexInvariant.hpp
    InteriorVertexInvariant.cpp
    InteriorEdgeInvariant.hpp
    InteriorEdgeInvariant.cpp
    MultiMeshLinkConditionInvariant.hpp
    MultiMeshLinkConditionInvariant.cpp
    MultiMeshMapValidInvariant.hpp
    MultiMeshMapValidInvariant.cpp

    MaxEdgeLengthInvariant.hpp
    MaxEdgeLengthInvariant.cpp
    MinEdgeLengthInvariant.hpp
    MinEdgeLengthInvariant.cpp

    MultiMeshTopologyInvariant.hpp
    MultiMeshTopologyInvariant.cpp
    MinIncidentValenceInvariant.hpp
    MinIncidentValenceInvariant.cpp
    TriMeshSubstructureTopologyPreservingInvariant.hpp
    TriMeshSubstructureTopologyPreservingInvariant.cpp
    TetMeshSubstructureTopologyPreservingInvariant.hpp
    TetMeshSubstructureTopologyPreservingInvariant.cpp
    SimplexInversionInvariant.hpp
    SimplexInversionInvariant.cpp
    TodoInvariant.hpp
    TodoInvariant.cpp
    ValenceImprovementInvariant.hpp
    ValenceImprovementInvariant.cpp

    EdgeValenceInvariant.hpp
    EdgeValenceInvariant.cpp

    NoBoundaryCollapseToInteriorInvariant.hpp
    NoBoundaryCollapseToInteriorInvariant.cpp

    MaxFunctionInvariant.hpp
    MaxFunctionInvariant.cpp

    FusionEdgeInvariant.hpp
    FusionEdgeInvariant.cpp


    internal/ConstantInvariant.hpp
    internal/ConstantInvariant.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
