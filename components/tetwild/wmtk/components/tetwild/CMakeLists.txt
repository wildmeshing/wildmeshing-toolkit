# incremental tetwild

set(COMPONENT_NAME tetwild)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    TetWildMesh.cpp
	TetWildMesh.h

	Parameters.h


	DelaunayBoxMesh.cpp
	VolumemesherInsertion.cpp
	read_tagged_mesh.hpp
	read_tagged_mesh.cpp

	tetwild.hpp
	tetwild.cpp
    )

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::data
	wmtk::shortest_edge_collapse
	jse::jse
)

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()
