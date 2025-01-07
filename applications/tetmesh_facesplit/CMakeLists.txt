include(wmtk_add_application)
wmtk_add_application(tetmesh_facesplit_app
        tetmesh_facesplit_main.cpp
        tetmesh_facesplit_spec.hpp
    )


target_link_libraries(shortest_edge_collapse_app PRIVATE
wmtk::input
wmtk::output
wmtk::multimesh
nlohmann_json::nlohmann_json
)

wmtk_register_integration_test(
    EXEC_NAME shortest_edge_collapse_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/tetmesh_facesplit_test_config.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG 23e95b46bacc9c962f6103f42f5f3e5da536942a)
