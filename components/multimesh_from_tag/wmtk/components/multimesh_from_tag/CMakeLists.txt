
set(SRC_FILES
    internal/MultiMeshFromTag.hpp
    internal/MultiMeshFromTag.cpp
    internal/MultiMeshFromTagOptions.hpp
    internal/MultiMeshFromTagOptions.cpp
	multimesh_from_tag.hpp
    multimesh_from_tag.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})