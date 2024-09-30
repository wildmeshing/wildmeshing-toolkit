wmtk_add_application(cdt_sec_app
    cdt_sec_main.cpp
    cdt_sec_spec.hpp
    )




# delaunay requires the input component and the delaunay component
target_link_libraries(cdt_sec_app PRIVATE
wmtk::input
wmtk::multimesh
wmtk::shortestedge_collapse
wmtk::cdt_optimization
wmtk::CDT
wmtk::output)

wmtk_register_integration_test(cdt_sec_app
    ${CMAKE_CURRENT_SOURCE_DIR}/cdt_sec_test_config.json
    "https://github.com/wildmeshing/data.git"
    e39994334263bd3fa8f6eb026d8487b5e9ae7191)
