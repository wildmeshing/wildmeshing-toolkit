# Sources
set(TEST_SOURCES
    substructure.cpp
    multimesh_topology.cpp
    tri_inversion.cpp
    tet_inversion.cpp
    )
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
