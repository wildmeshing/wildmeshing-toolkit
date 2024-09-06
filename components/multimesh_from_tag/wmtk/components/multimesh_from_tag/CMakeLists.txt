set(COMPONENT_NAME multimesh_from_tag)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    internal/MultiMeshFromTag.hpp
    internal/MultiMeshFromTag.cpp
	multimesh_from_tag.hpp
    multimesh_from_tag.cpp
    )


target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
