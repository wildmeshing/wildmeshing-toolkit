include(wmtk_add_application)
wmtk_add_application(shortest_edge_collapse_app
    shortest_edge_collapse_main.cpp
    shortest_edge_collapse_spec.hpp
    )


target_link_libraries(shortest_edge_collapse_app PRIVATE
wmtk::input
wmtk::shortest_edge_collapse
wmtk::output
nlohmann_json::nlohmann_json
)

wmtk_register_integration_test(
    EXEC_NAME shortest_edge_collapse_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/shortest_edge_collapse_test_config.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG 78158b6d3974b0fda6287737f43ca3c769ac7b8f)
