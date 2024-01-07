
set(SRC_FILES
    internal/DelaunayOptions.hpp
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
#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE geogram::delaunay_psm)
