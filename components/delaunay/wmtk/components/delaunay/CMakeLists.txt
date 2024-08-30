
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


add_component("delaunay")

include(delaunay_psm)
#COMPONENT_TARGET_NAME is set from the main cmake
target_sources(wmtk_delaunay PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_delaunay PRIVATE geogram::delaunay_psm)
