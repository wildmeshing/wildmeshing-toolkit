include(wmtk_add_application)
include(wmtk_register_jse_json)


#register_jse_json(APPLICATION_NAME isotropic_remeshing INPUT spec.json )

set(SHARED_STUFF
    make_multimesh.hpp
    make_multimesh.cpp
)


wmtk_add_application(isotropic_remeshing_3d_app
    main_3d.cpp
    #spec.hpp
    ${SHARED_STUFF}
    )

wmtk_add_application(isotropic_remeshing_uv_app
    main_uv.cpp
    #spec.hpp
    ${SHARED_STUFF}
    )


# isotropic_remeshing requires the input component and the procedural component
target_link_libraries(isotropic_remeshing_3d_app PRIVATE
wmtk::input
wmtk::isotropic_remeshing
wmtk::multimesh
wmtk::output
wmtk::application_utils
)

target_link_libraries(isotropic_remeshing_uv_app PRIVATE
wmtk::input
wmtk::isotropic_remeshing
wmtk::output
wmtk::multimesh
wmtk::application_utils
)

wmtk_register_integration_test(
    EXEC_NAME isotropic_remeshing_3d_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/test_config_3d.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG 6f171d11283d2977913a1b4e6129f9808357baf9
    CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}
)

wmtk_register_integration_test(
    EXEC_NAME isotropic_remeshing_uv_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/test_config_uv.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG 6f171d11283d2977913a1b4e6129f9808357baf9
    CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}
)
