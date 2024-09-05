wmtk_add_application(tw_simplification_app
    tw_simplification_main.cpp
    tw_simplification_spec.hpp
    )




# delaunay requires the input component and the delaunay component
target_link_libraries(tw_simplification_app PRIVATE
wmtk::input
wmtk::tetwild_simplification
wmtk::output)

wmtk_download_test_data(tetwild_simplification
    tw_simplification_app
    "https://github.com/wildmeshing/data.git"
    5a74c7ffda3bf1014875944f59e42a3db39775df)