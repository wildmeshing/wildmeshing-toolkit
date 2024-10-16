wmtk_add_application(tetwild_app
    tetwild_main.cpp
    tetwild_spec.hpp
    )

target_link_libraries(tetwild_app PRIVATE
wmtk::input
wmtk::tetwild_simplification
wmtk::to_points
wmtk::delaunay
wmtk::triangle_insertion
wmtk::wildmeshing
wmtk::winding_number
wmtk::output)