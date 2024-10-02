set(COMPONENT_NAME shortestedge_collapse)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    shortestedge_collapse.hpp
    shortestedge_collapse.cpp
    ShortestEdgeCollapseOptions.hpp
    )


target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
