include(wmtk_add_application)
wmtk_add_application(longest_edge_split_app
    longest_edge_split_main.cpp
    longest_edge_split_spec.hpp
    )


target_link_libraries(longest_edge_split_app PRIVATE
wmtk::input
wmtk::longest_edge_split
wmtk::multimesh
wmtk::output
nlohmann_json::nlohmann_json
)

wmtk_register_integration_test(
    EXEC_NAME longest_edge_split_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/longest_edge_split_test_config.json
)
