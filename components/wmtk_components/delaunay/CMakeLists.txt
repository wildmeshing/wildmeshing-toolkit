
set(SRC_FILES
    internal/DelaunayOptions.hpp
    internal/delaunay_2d.hpp
    internal/delaunay_2d.cpp
    internal/delaunay_3d.hpp
    internal/delaunay_3d.cpp
	delaunay.hpp
    delaunay.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
