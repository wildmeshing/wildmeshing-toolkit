wmtk_add_application(triwild_app
    triwild_main.cpp
    triwild_spec.hpp
    )

target_link_libraries(triwild_app PRIVATE
wmtk::input
wmtk::procedural
wmtk::edge_insertion
wmtk::wildmeshing
wmtk::winding_number
wmtk::output)