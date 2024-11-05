set(COMPONENT_NAME delaunay)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    internal/delaunay_2d.hpp
    internal/delaunay_2d.cpp
    internal/delaunay_3d.hpp
    internal/delaunay_3d.cpp
    internal/delaunay_geogram.hpp
    internal/delaunay_geogram.cpp
	delaunay.hpp
    delaunay.cpp
    )



include(delaunay_psm)
#COMPONENT_TARGET_NAME is set from the main cmake
target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE geogram::delaunay_psm)
