set(SOURCES
	AMIPS.cpp
	AMIPS.h

	TetMesh.h
	TetMesh.cpp

	Tuple.cpp

	EdgeCollapsingConn.cpp
	EdgeSplittingConn.cpp

	SwapMeshConnectivity.cpp

#	TriMesh.h
#	TriMesh.cpp
)

if(WILDMESHING_TOOLKIT_WITH_TBB)
	list(APPEND SOURCES

		ConcurrentTetMesh.h
		ConcurrentTetMesh.cpp
	)
endif()

prepend_current_path(SOURCES)
wildmeshing_toolkit_copy_headers(${SOURCES})
wildmeshing_toolkit_set_source_group(${SOURCES})
target_sources(WildMeshingToolkit PRIVATE ${SOURCES})

################################################################################
# Subfolders
################################################################################

add_subdirectory(utils)