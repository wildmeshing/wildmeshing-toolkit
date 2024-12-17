# Sources
set(TEST_SOURCES
    test_accessor.cpp
    tuple_accessor.cpp
    compound_accessor.cpp

    performance.cpp
    old_wmtk_attributecollection.cpp

    transaction_stack.cpp

    attribute_type.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})

include(libigl)
