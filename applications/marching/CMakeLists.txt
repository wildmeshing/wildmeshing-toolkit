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

wmtk_register_integration_test(
    EXEC_NAME marching_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/marching_test_config.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG 9014372217bb5170613ccdcec8759cf58583672c)
