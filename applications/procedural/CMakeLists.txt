include(wmtk_add_application)
include(wmtk_register_jse_json)
wmtk_add_application(procedural_app
    main.cpp
    spec.hpp
    )

    register_jse_json(APPLICATION_NAME procedural INPUT procedural_spec.json )

# procedural requires the input component and the procedural component
target_link_libraries(procedural_app PRIVATE
wmtk::input
wmtk::procedural
wmtk::output
)
