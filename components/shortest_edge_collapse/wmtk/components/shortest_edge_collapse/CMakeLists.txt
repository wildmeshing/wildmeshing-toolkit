set(COMPONENT_NAME shortest_edge_collapse)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    shortest_edge_collapse.hpp
    shortest_edge_collapse.cpp
    ShortestEdgeCollapseOptions.hpp
    )


target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
