set(SRC_FILES
    Embedding.hpp
    Embedding.cpp
    SubMesh.hpp
    SubMesh.cpp

    utils/write.hpp
    utils/write.cpp
    utils/submesh_from_multimesh.hpp
    utils/submesh_from_multimesh.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})