wmtk_add_application(wildmeshing_app
    wildmeshing_main.cpp
    )

target_link_libraries(wildmeshing_app PRIVATE
wmtk::input
wmtk::tetwild_simplification
wmtk::to_points
wmtk::delaunay
wmtk::triangle_insertion
wmtk::wildmeshing
wmtk::winding_number
wmtk::output)