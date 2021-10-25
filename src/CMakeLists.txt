set(SOURCES
		common.h

#		Mesh_old.h
		Mesh.h

		Parameters.h
		Envelope.h
)

prepend_current_path(SOURCES)
float_tetwild_copy_headers(${SOURCES})
float_tetwild_set_source_group(${SOURCES})
target_sources(FloatTetwild PRIVATE ${SOURCES})

################################################################################
# Subfolders
################################################################################

#add_subdirectory(external)
