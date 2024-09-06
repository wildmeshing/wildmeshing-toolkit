include(wmtk_add_application)
wmtk_add_application(procedural_app
    main.cpp
    spec.hpp
    )


# procedural requires the input component and the procedural component
target_link_libraries(procedural_app PRIVATE
wmtk::input
wmtk::procedural
wmtk::output
)
