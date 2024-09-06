set(COMPONENT_NAME marching)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    internal/Marching.hpp
    internal/Marching.cpp
    MarchingOptions.hpp
	marching.hpp
    marching.cpp
    )



target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

add_component_test(wmtk::${COMPONENT_NAME} tests/marching.cpp)
