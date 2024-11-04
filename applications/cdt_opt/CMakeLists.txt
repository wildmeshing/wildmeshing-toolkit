wmtk_add_application(cdt_opt_app
    cdt_opt_main.cpp
    cdt_opt_spec.hpp
    )



target_link_libraries(cdt_opt_app PRIVATE
wmtk::input
wmtk::multimesh
wmtk::cdt_optimization
wmtk::shortest_edge_collapse
wmtk::CDT
wmtk::wildmeshing
wmtk::output)

wmtk_register_integration_test(EXEC_NAME cdt_opt_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cdt_opt_test_config.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG e39994334263bd3fa8f6eb026d8487b5e9ae7191)
