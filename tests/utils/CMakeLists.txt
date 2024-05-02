
# Sources
set(TEST_SOURCES
    test_merkle.cpp
    test_insertion.cpp
    test_eigenmatrixwriter.cpp
    random.cpp
    test_point_inside_orient_2d.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
