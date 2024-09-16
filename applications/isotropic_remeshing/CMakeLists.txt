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
