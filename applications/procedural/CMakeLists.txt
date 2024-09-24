include(wmtk_add_application)
include(wmtk_register_jse_json)
wmtk_add_application(procedural_app
    main.cpp
    spec.hpp
    )

    #register_jse_json(APPLICATION_NAME procedural INPUT procedural_spec.json )

# procedural requires the input component and the procedural component
target_link_libraries(procedural_app PRIVATE
wmtk::input
wmtk::procedural
wmtk::output
)

wmtk_register_integration_test(
    EXEC_NAME procedural_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/test_config.json
    CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}
)
