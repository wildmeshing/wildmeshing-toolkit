include(wmtk_add_application)
include(wmtk_link_components)
wmtk_add_application(delaunay_app main.cpp)




# delaunay requires the input component and the delaunay component
wmtk_link_components(delaunay_app input delaunay output)
