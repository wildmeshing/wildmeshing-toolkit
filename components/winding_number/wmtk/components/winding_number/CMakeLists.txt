set(COMPONENT_NAME winding_number)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()


set(SRC_FILES
    internal/winding_number.hpp
    internal/winding_number.cpp
    winding_number.hpp
    winding_number.cpp)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE igl::core)