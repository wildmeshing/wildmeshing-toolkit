include(wmtk_add_application)
include(wmtk_register_jse_json)
wmtk_add_application(procedural_app
    main.cpp
    spec.hpp
    )

target_compile_features(procedural_app PUBLIC cxx_std_20)
    #register_jse_json(APPLICATION_NAME procedural INPUT procedural_spec.json )

# procedural requires the output component and the procedural component
target_link_libraries(procedural_app PRIVATE
wmtk::procedural
wmtk::output
wmtk::application_utils
)

#wmtk_register_integration_test(
#    EXEC_NAME procedural_app
#    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/test_config.json
#    CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}/examples
#)
