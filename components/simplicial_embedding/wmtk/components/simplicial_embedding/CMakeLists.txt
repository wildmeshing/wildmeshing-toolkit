set(COMPONENT_NAME simplicial_embedding)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    SimplicialEmbeddingOptions.hpp
    internal/SimplicialEmbedding.hpp
    internal/SimplicialEmbedding.cpp
	simplicial_embedding.hpp
    simplicial_embedding.cpp
    )


target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

add_component_test(wmtk::${COMPONENT_NAME} 
    ${PROJECT_SOURCE_DIR}/components/tests/simplicial_embedding/test_simplicial_embedding.cpp)