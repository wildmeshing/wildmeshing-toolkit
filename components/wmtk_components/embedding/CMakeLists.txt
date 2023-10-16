
set(SRC_FILES
    internal/EmbeddingOptions.hpp
    internal/Embedding.hpp
    internal/Embedding.cpp
    embedding.hpp
    embedding.cpp
    )
    
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
