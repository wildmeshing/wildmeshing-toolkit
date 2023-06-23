
set(SRC_FILES
    Accessor.cpp
    Accessor.hpp
    AttributeHandle.hpp
    Mesh.cpp
    Mesh.hpp
    MeshAttributes.cpp
    MeshAttributes.hpp
    Primitive.hpp
    Tuple.h
)
message(DEBUG "SRC_FILES: ${SRC_FILES}")
target_sources(wildmeshing_toolkit PRIVATE ${SOURCES})
