wmtk_add_application(delaunay_app
    delaunay_main.cpp
    delaunay_spec.hpp
    )




# delaunay requires the input component and the delaunay component
target_link_libraries(delaunay_app PRIVATE
wmtk::input
wmtk::delaunay
wmtk::output
wmtk::to_points)

wmtk_register_integration_test(delaunay_app
    ${CMAKE_CURRENT_SOURCE_DIR}/delaunay_test_config.json
    "https://github.com/wildmeshing/data.git"
    152a561697a6e923451ca8535309cbe1e116a9fa)
