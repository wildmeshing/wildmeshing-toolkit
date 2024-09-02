set(SRC_FILES
    AMIPSOptimizationSmoothing.hpp
    AMIPSOptimizationSmoothing.cpp

    AMIPSOptimizationSmoothingPeriodic.hpp
    AMIPSOptimizationSmoothingPeriodic.cpp

    AttributesUpdate.hpp
    AttributesUpdate.cpp

    Operation.hpp
    Operation.cpp

    OperationSequence.hpp
    OperationSequence.cpp

    AndOperationSequence.hpp
    AndOperationSequence.cpp

    OrOperationSequence.hpp
    OrOperationSequence.cpp

    MinOperationSequence.hpp
    MinOperationSequence.cpp

    #MeshOperation.hpp
    #MeshOperation.cpp

    EdgeCollapse.hpp
    EdgeCollapse.cpp

    EdgeSplit.hpp
    EdgeSplit.cpp

    EdgeOperationData.hpp
    EdgeOperationData.cpp

    OptimizationSmoothing.cpp
    OptimizationSmoothing.hpp

    Rounding.hpp
    Rounding.cpp

    TetWildTangentialLaplacianSmoothing.hpp
    TetWildTangentialLaplacianSmoothing.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(attribute_new)
add_subdirectory(attribute_update)
#
add_subdirectory(edge_mesh)
add_subdirectory(tri_mesh)
##TODODIscuss
add_subdirectory(tet_mesh)
#
add_subdirectory(composite)
#
add_subdirectory(utils)
#
add_subdirectory(internal)
