# manifold extraction

set(COMPONENT_NAME manifold_extraction)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
	return()
endif()

set(SRC_FILES
	ManExtractTetMesh.cpp
	ManExtractTetMesh.h

	Parameters.h

	manifold_extraction.cpp
	manifold_extraction.hpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::simwild
	wmtk::topological_offset
	wmtk_topological_offset_spec
	igl::predicates
	jse::jse
)

# #####################
# # TESTS
# #####################
if(${WMTK_COMPONENT_TESTS})
	add_subdirectory(tests)
endif()
