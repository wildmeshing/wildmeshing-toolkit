# incremental tetwild

include(gmp)
include(fenvelope)
include(volumemesher)

set(COMPONENT_NAME tetwild)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    TetWildMesh.cpp
	TetWildMesh.h

	common.h
	common.cpp
	Parameters.h

	EdgeSplitting.cpp
	EdgeCollapsing.cpp
	EdgeSwapping.cpp
	Smooth.cpp

	TriangleInsertion.cpp
	VolumemesherInsertion.cpp

	tetwild.hpp
	tetwild.cpp
	tetwild_spec.hpp
    )

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	Tracy::TracyClient
	wmtk::toolkit
	wmtk::data
	wmtk::sec_lib
	wmtk::remeshing_lib
	igl::predicates
	gmp::gmp
	VolumeRemesher::VolumeRemesher
	jse::jse
)

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()

