wmtk_add_application(simplicial_embedding_app
    simplicial_embedding_main.cpp
    simplicial_embedding_spec.hpp
    )




# delaunay requires the input component and the delaunay component
target_link_libraries(simplicial_embedding_app PRIVATE
wmtk::input
wmtk::simplicial_embedding
wmtk::output)

wmtk_register_integration_test(
    EXEC_NAME simplicial_embedding_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simplicial_embedding_test_config.json
)
