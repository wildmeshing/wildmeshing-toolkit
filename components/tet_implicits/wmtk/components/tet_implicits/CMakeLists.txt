# incremental tet_implicits

set(COMPONENT_NAME tet_implicits)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    TetImplicitsMesh.cpp
	TetImplicitsMesh.h

	Parameters.h

	EdgeSplitting.cpp

	TetImplicitsMeshInit.cpp

	tet_implicits.hpp
	tet_implicits.cpp
	tet_implicits_spec.hpp

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

# if(${WMTK_COMPONENT_TESTS})
#     add_subdirectory(tests)
# endif()

