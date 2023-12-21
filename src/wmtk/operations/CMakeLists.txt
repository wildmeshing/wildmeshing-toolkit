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

    OptimizationSmoothing.cpp
    OptimizationSmoothing.hpp

    VertexLaplacianSmooth.hpp
    VertexLaplacianSmooth.cpp

    # VertexTangentialLaplacianSmooth.hpp
    # VertexTangentialLaplacianSmooth.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(data)
add_subdirectory(composite)

add_subdirectory(utils)

