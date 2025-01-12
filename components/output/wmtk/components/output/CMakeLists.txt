set(COMPONENT_NAME output)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    output.cpp
	output.hpp
    OutputOptions.hpp
    OutputOptions.cpp
    utils/format.hpp
    )



target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC wmtk::multimesh) 
