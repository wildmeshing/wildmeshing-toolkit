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

	extract_soup.hpp
	extract_soup.cpp
	
	EmbedSurface.hpp
	EmbedSurface.cpp

	Parameters.h

	EdgeSplitting.cpp
	EdgeCollapsing.cpp
	EdgeSwapping.cpp
	Smooth.cpp

	TriangleInsertion.cpp
	VolumemesherInsertion.cpp

	image_simulation.hpp
	image_simulation.cpp
	image_simulation_spec.hpp

	read_image_msh.hpp
	read_image_msh.cpp
    )

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	Tracy::TracyClient
	wmtk::toolkit
	wmtk::data
	wmtk::sec_lib
	igl::predicates
	VolumeRemesher::VolumeRemesher
	jse::jse
)

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()

