set(SOURCES
	TetMesh.h
	TetMesh.cpp

	Tuple.cpp


	TriMesh.h
	TriMesh.cpp
	Tuple.cpp

	EdgeCollapsingConn.cpp
	EdgeSplittingConn.cpp

	SwapMeshConnectivity.cpp

	ConcurrentTetMesh.h
	ConcurrentTetMesh.cpp
)

prepend_current_path(SOURCES)
wildmeshing_toolkit_set_source_group(${SOURCES})
target_sources(WildMeshingToolkit PRIVATE ${SOURCES})

################################################################################
# Subfolders
################################################################################

add_subdirectory(utils)
