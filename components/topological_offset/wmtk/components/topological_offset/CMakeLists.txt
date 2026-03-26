# topological offset

set(COMPONENT_NAME topological_offset)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
	TopoOffsetTetMesh.h
    TopoOffsetTetMesh.cpp

	TopoOffsetTriMesh.h
	TopoOffsetTriMesh.cpp

	Parameters.h
	
	ComponentSplitting.cpp
	SimplicialComplexBVH.hpp
	Spatial.cpp
	Circle.hpp
	Sphere.hpp

	topological_offset.hpp
	topological_offset.cpp
	topological_offset_spec.hpp

	read_image_msh.hpp
	read_image_msh.cpp
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

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()
