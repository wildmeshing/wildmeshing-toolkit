
# Sources
set(TEST_SOURCES
    test_merkle.cpp
    test_eigenmatrixwriter.cpp
    random.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})

if(WMTK_ENABLE_MULTIMESH)
target_sources(wmtk_tests PRIVATE test_insertion.cpp)
endif()
