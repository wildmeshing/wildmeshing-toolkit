include(wmtk_add_application)
include(wmtk_register_jse_json)


#register_jse_json(APPLICATION_NAME isotropic_remeshing INPUT spec.json )


wmtk_add_application(isotropic_remeshing_app
    main.cpp
    spec.hpp
    )


# isotropic_remeshing requires the input component and the procedural component
target_link_libraries(isotropic_remeshing_app PRIVATE
wmtk::input
wmtk::isotropic_remeshing
wmtk::output
)

wmtk_register_integration_test(
    EXEC_NAME isotropic_remeshing_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/test_config.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG 152a561697a6e923451ca8535309cbe1e116a9fa
    CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}
)
