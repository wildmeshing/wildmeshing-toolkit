
set(SRC_FILES
    internal/MeshInfoOption.h
    mesh_info.h)
message(DEBUG "SRC_FILES: ${SRC_FILES}")
target_sources(wildmeshing_components PRIVATE ${SOURCES})