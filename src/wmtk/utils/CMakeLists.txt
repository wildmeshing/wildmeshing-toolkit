
set(SRC_FILES
    Logger.cpp
    Logger.hpp
    trimesh_topology_initialization.cpp
    trimesh_topology_initialization.h
)
message(DEBUG "SRC_FILES: ${SRC_FILES}")
target_sources(wildmeshing_toolkit PRIVATE ${SOURCES})
