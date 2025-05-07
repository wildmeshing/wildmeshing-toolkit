wmtk_add_application(c1_meshing_split_app
    c1_meshing_split_main.cpp
    c1_meshing_split_spec.hpp
    )



target_link_libraries(c1_meshing_split_app PRIVATE
wmtk::input
wmtk::winding_number
wmtk::output
igl::core)

# wmtk_register_integration_test(EXEC_NAME c1_meshing_split_app
#     CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/c1_meshing_split_test_config.json
#     GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
#     GIT_TAG d1063ed50de45a1bcae9f8b6ae9b8b1d42885abe)
