# topological offset

set(COMPONENT_NAME topological_offset)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
	return()
endif()

set(SRC_FILES
	TopoOffsetTetMesh.h
	TagEnvelopes.hpp
	TopoOffsetTetMesh.cpp
	Optimize2d.cpp
	Smooth.cpp
	Collapse.cpp
	Swap.cpp

	TopoOffsetTriMesh.h
	TopoOffsetTriMesh.cpp

	Parameters.h

	EdgeSplittingTet.cpp
	EdgeSplittingTri.cpp
	SimplicialComplexBVH.hpp
	Spatial.cpp
	Circle.hpp
	Sphere.hpp

	OffsetPotential.hpp
	OffsetPotential.cpp

	topological_offset.hpp
	topological_offset.cpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

# The smooth offset potential. Deliberately linked HERE and not into wmtk::toolkit: this is the
# only consumer, and ipc-toolkit brings geogram, filib, TBB and tight_inclusion with it, none of
# which tetwild / triwild / simwild have any use for.
include(ipc_toolkit)

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::simwild
	jse::jse
	ipc::toolkit
)

# #####################
# # TESTS
# #####################
if(${WMTK_COMPONENT_TESTS})
	add_subdirectory(tests)
endif()
