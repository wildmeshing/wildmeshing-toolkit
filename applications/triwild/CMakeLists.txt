wmtk_add_application(triwild_app
    triwild_main.cpp
    triwild_spec.hpp
    triwild_grid.hpp
    triwild_grid.cpp
    )

target_link_libraries(triwild_app PRIVATE
wmtk::input
wmtk::procedural
wmtk::edge_insertion
wmtk::wildmeshing
wmtk::winding_number
wmtk::output)

wmtk_register_integration_test(EXEC_NAME triwild_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/triwild_test_config.json
)
