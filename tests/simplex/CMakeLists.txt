# Sources
set(TEST_SOURCES
    test_boundary_with_preserved_face.cpp
    test_simplex_collection.cpp
    comparison.cpp
    faces.cpp
    raw_simplex_collection.cpp
    boundary.cpp
    cofaces.cpp
    test_utils.hpp
    open_star.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
