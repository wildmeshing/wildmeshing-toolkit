wmtk_add_application(tetmesh_surface_facesplit_app
        tetmesh_surface_facesplit_main.cpp
        tetmesh_surface_facesplit_spec.hpp
    )



target_link_libraries(tetmesh_surface_facesplit_app PRIVATE
wmtk::input
wmtk::multimesh
wmtk::output)

# wmtk_register_integration_test(EXEC_NAME tetmesh_surface_facesplit_app
#     CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/tetmesh_surface_facesplit_test_config.json
#     GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
#     GIT_TAG d1063ed50de45a1bcae9f8b6ae9b8b1d42885abe)
