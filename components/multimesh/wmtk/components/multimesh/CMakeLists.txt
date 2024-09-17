set(COMPONENT_NAME multimesh)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    multimesh.hpp
    multimesh.cpp
    )


target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
