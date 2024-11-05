set(COMPONENT_NAME edge_insertion)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    internal/EdgeInsOptions.hpp
    edge_insertion.hpp
    edge_insertion.cpp
    internal/edge_insertion.hpp
    internal/edge_insertion.cpp)



target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE wmtk::multimesh_from_tag)

target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE igl::core)
