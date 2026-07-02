include(volumemesher)

set(COMPONENT_NAME simwild)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    SimWildMesh.cpp
	SimWildMesh.h

	EmbedSurface.hpp
	EmbedSurface.cpp

	Parameters.h

	EdgeSplitting.cpp
	EdgeCollapsing.cpp
	EdgeSwapping.cpp
	Smooth.cpp
	
	VolumemesherInsertion.cpp

	simwild.hpp
	simwild.cpp

	read_image_msh.hpp
	read_image_msh.cpp

	SimWildMeshTri.hpp
	SimWildMeshTri.cpp
	ConnectedComponent.hpp
	AnnotationsTri.cpp
	AnnotationsTet.cpp

	expression_parser/Expression.hpp
	expression_parser/Parser.hpp
	expression_parser/Parser.cpp
    )

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::data
	wmtk::shortest_edge_collapse
	VolumeRemesher::VolumeRemesher
	jse::jse
)

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()

