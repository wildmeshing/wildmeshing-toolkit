
# Sources
set(TEST_SOURCES
    test_merkle.cpp
    test_insertion.cpp
    test_eigenmatrixwriter.cpp
    test_rational.cpp
    random.cpp
    dynamic_array.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
