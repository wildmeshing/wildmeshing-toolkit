# manifold extraction

set(COMPONENT_NAME manifold_extraction)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    ManExtractMesh.cpp
	ManExtractMesh.h

	Parameters.h
	ComponentSplitting.cpp

	manifold_extraction.hpp
	manifold_extraction.cpp
	manifold_extraction.hpp

	read_image_msh.hpp
	read_image_msh.cpp
	write_image_msh.hpp
	write_image_msh.cpp

	file_generation.cpp
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
