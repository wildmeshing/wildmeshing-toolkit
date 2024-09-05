set(COMPONENT_NAME triangle_insertion)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    triangle_insertion.hpp
    triangle_insertion.cpp)



target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE wmtk::multimesh_from_tag)
