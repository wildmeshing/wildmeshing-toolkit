include(wmtk_add_application)
wmtk_add_application(marching_app
    marching_main.cpp
    marching_spec.hpp
    )


# marching requires the input component and the marching component
target_link_libraries(marching_app PRIVATE
wmtk::input
wmtk::marching
wmtk::output 
nlohmann_json::nlohmann_json
)
