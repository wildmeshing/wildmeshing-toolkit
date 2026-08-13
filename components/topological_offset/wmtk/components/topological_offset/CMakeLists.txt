# topological offset

set(COMPONENT_NAME topological_offset)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
	return()
endif()

set(SRC_FILES
	TopoOffsetTetMesh.h
	TopoOffsetTetMesh.cpp
	Optimize2d.cpp
	Smooth.cpp
	Collapse.cpp
	Swap.cpp
	Quadrics.hpp
	Quadrics.cpp

	TopoOffsetTriMesh.h
	TopoOffsetTriMesh.cpp

	Parameters.h

	EdgeSplittingTet.cpp
	EdgeSplittingTri.cpp
	SimplicialComplexBVH.hpp
	Spatial.cpp
	Circle.hpp
	Sphere.hpp

	topological_offset.hpp
	topological_offset.cpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::simwild
	jse::jse
)

# #####################
# # TESTS
# #####################
if(${WMTK_COMPONENT_TESTS})
	add_subdirectory(tests)
endif()
