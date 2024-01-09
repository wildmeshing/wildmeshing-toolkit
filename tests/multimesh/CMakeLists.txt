# Sources
set(TEST_SOURCES
    test_create_tag.cpp
    test_extract_child_mesh.cpp
    test_multi_mesh_visitor.cpp
    test_multi_mesh.cpp
    test_multi_mesh_rollback.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
