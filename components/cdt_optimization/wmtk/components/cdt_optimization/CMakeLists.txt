set(COMPONENT_NAME cdt_optimization)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    cdt_optimization.hpp
    cdt_optimization.cpp
    )


target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
