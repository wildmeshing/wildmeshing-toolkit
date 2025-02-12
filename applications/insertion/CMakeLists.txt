wmtk_add_application(insertion_app
    insertion_main.cpp
    insertion_spec.hpp
    )




# insertion requires the input component and the insertion component
target_link_libraries(insertion_app PRIVATE
wmtk::input
wmtk::triangle_insertion
wmtk::output)

wmtk_register_integration_test(
    EXEC_NAME insertion_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/insertion_test_config.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG 6f171d11283d2977913a1b4e6129f9808357baf9)
