# incremental image_simulation

include(volumemesher)

set(COMPONENT_NAME image_simulation)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    ImageSimulationMesh.cpp
	ImageSimulationMesh.h

	EmbedSurface.hpp
	EmbedSurface.cpp

	Parameters.h

	EdgeSplitting.cpp
	EdgeCollapsing.cpp
	EdgeSwapping.cpp
	Smooth.cpp
	
	VolumemesherInsertion.cpp

	image_simulation.hpp
	image_simulation.cpp

	read_image_msh.hpp
	read_image_msh.cpp

	ImageSimulationMeshTri.hpp
	ImageSimulationMeshTri.cpp
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

