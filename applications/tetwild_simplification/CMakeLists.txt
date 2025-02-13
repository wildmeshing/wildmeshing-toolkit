wmtk_add_application(tw_simplification_app
    tw_simplification_main.cpp
    tw_simplification_spec.hpp
    )




# delaunay requires the input component and the delaunay component
target_link_libraries(tw_simplification_app PRIVATE
wmtk::input
wmtk::tetwild_simplification
wmtk::output)

wmtk_register_integration_test(
    EXEC_NAME tw_simplification_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/tw_simplification_test_config.json
)
