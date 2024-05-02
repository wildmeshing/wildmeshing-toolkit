# Sources
set(TEST_SOURCES
    test_accessor.cpp
    tuple_accessor.cpp
    compound_accessor.cpp
    hybrid_rational_accessor.cpp

    performance.cpp
    caches.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
