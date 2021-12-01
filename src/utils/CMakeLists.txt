set(SOURCES
	Logger.cpp
	Logger.hpp

    Delaunay.cpp
    Delaunay.hpp
    Delaunay/Delaunay_psm.cpp
    Delaunay/Delaunay_psm.h

	DisableWarnings.hpp
	EnableWarnings.hpp

	TupleUtils.cpp
	TupleUtils.hpp

	VectorUtils.h

)

prepend_current_path(SOURCES)
wildmeshing_toolkit_copy_headers(${SOURCES})
wildmeshing_toolkit_set_source_group(${SOURCES})
target_sources(WildMeshingToolkit PRIVATE ${SOURCES})

################################################################################
# Subfolders
################################################################################

