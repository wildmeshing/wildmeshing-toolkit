include(wmtk_add_application)
include(wmtk_register_jse_json)
wmtk_add_application(fuse_with_tags_app
    main.cpp
    run.hpp
    run.cpp

    utils.cpp
    utils.hpp
    fuse_eigen.hpp
    fuse_eigen.cpp
    fuse.hpp
    fuse.cpp

    read_vid_pairs.hpp
    read_vid_pairs.cpp

    edges_to_tuples.hpp
    edges_to_tuples.cpp
    )

target_compile_features(fuse_with_tags_app PUBLIC cxx_std_20)

# fuse_with_tags requires the output component and the convert component
target_link_libraries(fuse_with_tags_app PRIVATE
wmtk::input
wmtk::output
wmtk::multimesh
wmtk::tetwild_simplification
igl::core
wmtk::application_utils
)

