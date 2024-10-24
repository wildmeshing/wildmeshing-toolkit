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

    uvEdgeInvariant.hpp
    uvEdgeInvariant.cpp

    internal/ConstantInvariant.hpp
    internal/ConstantInvariant.cpp

    Swap44EnergyBeforeInvariant.hpp
    Swap44EnergyBeforeInvariant.cpp
    Swap32EnergyBeforeInvariant.hpp
    Swap32EnergyBeforeInvariant.cpp
    Swap23EnergyBeforeInvariant.hpp
    Swap23EnergyBeforeInvariant.cpp
    Swap56EnergyBeforeInvariant.hpp
    Swap56EnergyBeforeInvariant.cpp

    Swap44EnergyBeforeInvariantDouble.hpp
    Swap44EnergyBeforeInvariantDouble.cpp
    Swap32EnergyBeforeInvariantDouble.hpp
    Swap32EnergyBeforeInvariantDouble.cpp
    Swap56EnergyBeforeInvariantDouble.hpp
    Swap56EnergyBeforeInvariantDouble.cpp

    SeparateSubstructuresInvariant.hpp
    SeparateSubstructuresInvariant.cpp

    RoundedInvariant.hpp
    RoundedInvariant.cpp

    CollapseEnergyBeforeInvariant.hpp
    CollapseEnergyBeforeInvariant.cpp
    CollapseSoftEnergyBeforeInvariant.hpp
    CollapseSoftEnergyBeforeInvariant.cpp
    CollapseEnergyBeforeInvariantDouble.hpp
    CollapseEnergyBeforeInvariantDouble.cpp

    NoChildMeshAttachingInvariant.hpp
    NoChildMeshAttachingInvariant.cpp

    EnergyFilterInvariant.hpp
    EnergyFilterInvariant.cpp

    FrozenVertexInvariant.hpp
    FrozenVertexInvariant.cpp
    
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
