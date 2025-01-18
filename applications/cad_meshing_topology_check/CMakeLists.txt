include(wmtk_add_application)
include(wmtk_register_jse_json)
wmtk_add_application(cad_meshing_topology_check_app
    main.cpp
    
    )

target_compile_features(cad_meshing_topology_check_app PUBLIC cxx_std_20)

# fuse_with_tags requires the output component and the convert component
target_link_libraries(cad_meshing_topology_check_app PRIVATE
wmtk::input
wmtk::output
wmtk::multimesh
igl::core
wmtk::application_utils
)

