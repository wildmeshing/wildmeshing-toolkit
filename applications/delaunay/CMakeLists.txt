wmtk_add_application(delaunay_app
    delaunay_main.cpp
    delaunay_spec.hpp
    )




# delaunay requires the input component and the delaunay component
target_link_libraries(delaunay_app PRIVATE
wmtk::input
wmtk::delaunay
wmtk::output
wmtk::to_points
wmtk::application_utils
)

wmtk_register_integration_test(
    EXEC_NAME delaunay_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/delaunay_test_config.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG 6f171d11283d2977913a1b4e6129f9808357baf9)
