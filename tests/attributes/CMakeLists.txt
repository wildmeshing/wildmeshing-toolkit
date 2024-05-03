# Sources
set(TEST_SOURCES
    test_accessor.cpp
    tuple_accessor.cpp
    compound_accessor.cpp
    hybrid_rational_accessor.cpp

    performance.cpp
    caches.cpp
    old_wmtk_attributecollection.cpp

    transaction_stack.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})

include(libigl)
