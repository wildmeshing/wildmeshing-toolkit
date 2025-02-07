set(SRC_FILES
    Embedding.hpp
    Embedding.cpp
    SubMesh.hpp
    SubMesh.cpp

    utils/write.hpp
    utils/write.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})