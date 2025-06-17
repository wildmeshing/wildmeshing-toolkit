set(COMPONENT_NAME simplicial_embedding)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
	SimplicialEmbeddingTriMesh.hpp
	SimplicialEmbeddingTriMesh.cpp

	simplicial_embedding.hpp
	simplicial_embedding.cpp
	simplicial_embedding_spec.hpp
    )

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	# additional libraries go here...
)

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()

