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

# wmtk_register_integration_test(cdt_sec_app
#     ${CMAKE_CURRENT_SOURCE_DIR}/delaunay_test_config.json
#     "https://github.com/wildmeshing/data.git"
#     152a561697a6e923451ca8535309cbe1e116a9fa)
