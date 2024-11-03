include(wmtk_add_application)
include(wmtk_register_jse_json)
wmtk_add_application(convert_app
    main.cpp
    )

target_compile_features(convert_app PUBLIC cxx_std_20)
    #register_jse_json(APPLICATION_NAME convert INPUT convert_spec.json )

# convert requires the output component and the convert component
target_link_libraries(convert_app PRIVATE
wmtk::input
wmtk::output
wmtk::multimesh
wmtk::application_utils
)

wmtk_register_integration_test(
    EXEC_NAME convert_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/test_config.json
    CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}/examples
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG 152a561697a6e923451ca8535309cbe1e116a9fa
    EXTRA_ARGUMENTS run
)
