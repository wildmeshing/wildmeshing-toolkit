# triwild

set(COMPONENT_NAME triwild)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    TriWildMesh.cpp
	TriWildMesh.h

	Parameters.h

	EdgeSplitting.cpp
	EdgeCollapsing.cpp
	EdgeSwapping.cpp
	Smooth.cpp

	triwild.hpp
	triwild.cpp

	init_from_delaunay.hpp
	init_from_delaunay.cpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	jse::jse
)

######################
## TESTS
######################

# if(${WMTK_COMPONENT_TESTS})
#     add_subdirectory(tests)
# endif()

