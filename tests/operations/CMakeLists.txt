
# Sources
set(TEST_SOURCES
    collapse_2d.cpp
    construction_2d.cpp
    data_2d.cpp
    split_2d.cpp
    swap_2d.cpp
    vertex_optimization.cpp
    test_projection_operation.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
