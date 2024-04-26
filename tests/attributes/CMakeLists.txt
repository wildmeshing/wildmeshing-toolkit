# Sources
set(TEST_SOURCES
    test_accessor.cpp
    compound_accessor.cpp
    hybrid_rational_accessor.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})

if(WMTK_ENABLE_MULTIMESH)
target_sources(wmtk_tests PRIVATE 
    tuple_accessor.cpp
    )
endif()
