set(COMPONENT_NAME to_points)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()


set(SRC_FILES
    to_points.hpp
    to_points.cpp)


#COMPONENT_TARGET_NAME is set from the main cmake
target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
