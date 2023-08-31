# Sources
set(TEST_SOURCES
	test_2d_energy.cpp
    
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})

target_link_libraries(wmtk_tests PUBLIC
    wildmeshing_toolkit
)

