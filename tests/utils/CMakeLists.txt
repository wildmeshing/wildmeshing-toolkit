
# Sources
set(TEST_SOURCES
    test_merkle.cpp
    test_insertion.cpp
    test_eigenmatrixwriter.cpp
    test_edge_insertion.cpp
    test_rational.cpp
    random.cpp
    test_dynamic_array.cpp
    disjoint_set.cpp
    valence_checker.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
