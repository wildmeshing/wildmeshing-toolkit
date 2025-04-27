include(wmtk_add_application)
include(wmtk_register_jse_json)
wmtk_add_application(rgb_app
    main.cpp
    )

target_compile_features(rgb_app PUBLIC cxx_std_20)
    #register_jse_json(APPLICATION_NAME rgb INPUT rgb_spec.json )

# rgb requires the output component and the rgb component
target_link_libraries(rgb_app PRIVATE
wmtk::input
wmtk::output
wmtk::multimesh
wmtk::application_utils
)

wmtk_register_integration_test(
    EXEC_NAME rgb_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/test_config.json
    CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}/examples
    #EXTRA_ARGUMENTS run
)
