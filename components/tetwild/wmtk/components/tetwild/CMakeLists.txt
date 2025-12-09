# incremental tetwild

include(volumemesher)

set(COMPONENT_NAME tetwild)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    TetWildMesh.cpp
	TetWildMesh.h

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

	orig/Args.h
	orig/Common.cpp
	orig/Common.h
	orig/EdgeCollapser.cpp
	orig/EdgeCollapser.h
	orig/EdgeRemover.cpp
	orig/EdgeRemover.h
	orig/EdgeSplitter.cpp
	orig/EdgeSplitter.h
	orig/ForwardDecls.h
	orig/LocalOperations.cpp
	orig/LocalOperations.h
	orig/MeshRefinement.cpp
	orig/MeshRefinement.h
	orig/State.cpp
	orig/State.h
	orig/TetmeshElements.cpp
	orig/TetmeshElements.h
	orig/VertexSmoother.cpp
	orig/VertexSmoother.h
    )

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::data
	wmtk::sec_lib
	wmtk::remeshing_lib
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

