
set(SRC_FILES
    Mesh.cpp
    Mesh.hpp
    MeshAttributes.hpp
    MeshAttributes_impl.hpp
    Accessor.hpp
    Accessor_impl.hpp)
message(DEBUG "SRC_FILES: ${SRC_FILES}")
target_sources(wildmeshing_toolkit PRIVATE ${SOURCES})
