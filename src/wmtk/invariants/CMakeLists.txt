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

    MaxEdgeLengthInvariant.hpp
    MaxEdgeLengthInvariant.cpp
    MinEdgeLengthInvariant.hpp
    MinEdgeLengthInvariant.cpp

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


    RoundedInvariant.hpp
    RoundedInvariant.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

if(WMTK_ENABLE_MULTIMESH)
    target_sources(wildmeshing_toolkit PRIVATE 
    uvEdgeInvariant.hpp
    uvEdgeInvariant.cpp
        MultiMeshLinkConditionInvariant.hpp
        MultiMeshLinkConditionInvariant.cpp
        MultiMeshMapValidInvariant.hpp
        MultiMeshMapValidInvariant.cpp
        MultiMeshTopologyInvariant.hpp
        MultiMeshTopologyInvariant.cpp
        FusionEdgeInvariant.hpp
        FusionEdgeInvariant.cpp
    SeparateSubstructuresInvariant.hpp
    SeparateSubstructuresInvariant.cpp
        )
endif()
