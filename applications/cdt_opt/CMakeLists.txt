wmtk_add_application(cdt_opt_app
    cdt_opt_main.cpp
    cdt_opt_spec.hpp
    )



target_link_libraries(cdt_opt_app PRIVATE
wmtk::input
wmtk::multimesh
wmtk::CDT
wmtk::wildmeshing
wmtk::output)

wmtk_register_integration_test(EXEC_NAME cdt_opt_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cdt_opt_test_config.json)
