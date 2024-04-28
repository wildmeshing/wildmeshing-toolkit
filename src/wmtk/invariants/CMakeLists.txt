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

    SmallerFunctionInvariant.hpp
    SmallerFunctionInvariant.cpp
    uvEdgeInvariant.hpp
    uvEdgeInvariant.cpp

    internal/ConstantInvariant.hpp
    internal/ConstantInvariant.cpp

    Swap44EnergyBeforeInvariant.hpp
    Swap44EnergyBeforeInvariant.cpp
    Swap44_2EnergyBeforeInvariant.hpp
    Swap44_2EnergyBeforeInvariant.cpp
    Swap32EnergyBeforeInvariant.hpp
    Swap32EnergyBeforeInvariant.cpp
    Swap23EnergyBeforeInvariant.hpp
    Swap23EnergyBeforeInvariant.cpp

    SeparateSubstructuresInvariant.hpp
    SeparateSubstructuresInvariant.cpp

    RoundedInvariant.hpp
    RoundedInvariant.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
