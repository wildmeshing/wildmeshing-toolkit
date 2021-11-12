set(SOURCES
	Logger.cpp
	Logger.hpp

	DisableWarnings.hpp
	EnableWarnings.hpp

	VectorUtils.h

)

prepend_current_path(SOURCES)
wildmeshing_toolkit_copy_headers(${SOURCES})
wildmeshing_toolkit_set_source_group(${SOURCES})
target_sources(WildMeshingToolkit PRIVATE ${SOURCES})

################################################################################
# Subfolders
################################################################################

