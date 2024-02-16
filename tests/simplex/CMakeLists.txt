# Sources
set(TEST_SOURCES
    test_boundary_with_preserved_face.cpp
    test_face_coface_intersection.cpp
)
target_sources(wmtk_tests PRIVATE ${TEST_SOURCES})
