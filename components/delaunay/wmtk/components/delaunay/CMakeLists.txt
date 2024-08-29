
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
#COMPONENT_TARGET_NAME is set from the main cmake
target_sources(${COMPONENT_TARGET_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${COMPONENT_TARGET_NAME} PRIVATE wmtk::component_utils geogram::delaunay_psm)
