set(SRC_FILES
    TetMeshOperation.hpp
    TetMeshOperation.cpp
    VertexLaplacianSmoothWithTags.hpp
    VertexLaplacianSmoothWithTags.cpp
    VertexPushOffset.hpp
    VertexPushOffset.cpp
    VertexAttributesUpdateBase.hpp
    VertexAttributesUpdateBase.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
