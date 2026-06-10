# incremental tet_remeshing

set(COMPONENT_NAME tet_remeshing)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    TetRemeshingMesh.cpp
	TetRemeshingMesh.h

	Parameters.h

	EdgeSplitting.cpp
	EdgeCollapsing.cpp
	EdgeSwapping.cpp
	Smooth.cpp

	VolumemesherInsertion.cpp

	tet_remeshing.hpp
	tet_remeshing.cpp
	tet_remeshing_spec.hpp

	read_image_msh.hpp
	read_image_msh.cpp

	SurfaceMesh.hpp
	SurfaceMesh.cpp
    )

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	igl::predicates
	jse::jse
)

######################
## TESTS
######################

# if(${WMTK_COMPONENT_TESTS})
#     add_subdirectory(tests)
# endif()

