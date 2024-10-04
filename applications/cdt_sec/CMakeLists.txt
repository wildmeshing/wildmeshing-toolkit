wmtk_add_application(cdt_sec_app
    cdt_sec_main.cpp
    cdt_sec_spec.hpp
    )




# delaunay requires the input component and the delaunay component
target_link_libraries(cdt_sec_app PRIVATE
wmtk::input
wmtk::multimesh
wmtk::shortestedge_collapse
wmtk::CDT
wmtk::output)

wmtk_register_integration_test(EXEC_NAME cdt_sec_app
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cdt_sec_test_config.json
    GIT_REPOSITORY "https://github.com/wildmeshing/data.git"
    GIT_TAG e39994334263bd3fa8f6eb026d8487b5e9ae7191)
