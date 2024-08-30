include(wmtk_add_application)
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