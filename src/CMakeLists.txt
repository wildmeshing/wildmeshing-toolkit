set(SOURCES
		common.h

		TetMesh.h
		TetMesh.cpp

		Parameters.h
		Envelope.h

		TetWild.cpp
		TetWild.h

		AMIPS.cpp
		AMIPS.h
)

prepend_current_path(SOURCES)
float_tetwild_copy_headers(${SOURCES})
float_tetwild_set_source_group(${SOURCES})
target_sources(FloatTetwild PRIVATE ${SOURCES})

################################################################################
# Subfolders
################################################################################

#add_subdirectory(external)
