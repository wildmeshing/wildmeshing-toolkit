wmtk_add_application(triwild_submesh_app
    triwild_submesh_main.cpp
    triwild_submesh_spec.hpp
    triwild_grid.hpp
    triwild_grid.cpp
    )

target_link_libraries(triwild_submesh_app PRIVATE
wmtk::input
wmtk::procedural
wmtk::edge_insertion
wmtk::wildmeshing
wmtk::winding_number
wmtk::output)

# wmtk_register_integration_test(EXEC_NAME triwild_submesh_app
#     CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/triwild_test_config.json
#     GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
#     GIT_TAG 82ea9d55c901bbc86d48de39383e9c85d8f67686)
