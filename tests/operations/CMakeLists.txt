
# Sources
set(TEST_SOURCES
    collapse_2d.cpp
    construction_2d.cpp
    data_2d.cpp
    split_2d.cpp
    swap_2d.cpp
    split_1d.cpp
    collapse_1d.cpp
    test_attribute_transfer.cpp
    vertex_optimization.cpp
    laplacian_smoothing.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
