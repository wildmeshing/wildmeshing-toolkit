set(COMPONENT_NAME marching)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()
set(SRC_FILES
    internal/MarchingOptions.hpp
    internal/MarchingOptions.cpp
    internal/Marching.hpp
    internal/Marching.cpp
	marching.hpp
    marching.cpp
    )



target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE stb::image)

add_component_test(wmtk::${COMPONENT_NAME} tests/marching.cpp)
