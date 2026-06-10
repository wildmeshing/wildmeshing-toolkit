set(COMPONENT_NAME shortest_edge_collapse)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    ShortestEdgeCollapse.cpp
	ShortestEdgeCollapse.h

	shortest_edge_collapse.hpp
	shortest_edge_collapse.cpp
	shortest_edge_collapse_spec.hpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	# igl::predicates
)

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()

