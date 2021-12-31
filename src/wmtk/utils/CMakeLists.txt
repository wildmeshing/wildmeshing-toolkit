set(SOURCES
	AMIPS.cpp
	AMIPS.h

	Logger.cpp
	Logger.hpp

    Delaunay.cpp
    Delaunay.hpp

	DisableWarnings.hpp
	EnableWarnings.hpp

	TupleUtils.cpp
	TupleUtils.hpp

	VectorUtils.h

	TetraQualityUtils.hpp
	TetraQualityUtils.cpp
)

prepend_current_path(SOURCES)
wildmeshing_toolkit_set_source_group(${SOURCES})
target_sources(WildMeshingToolkit PRIVATE ${SOURCES})

################################################################################
# Subfolders
################################################################################

