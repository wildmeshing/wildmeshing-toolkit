#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <random>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/extract_subset/extract_subset.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"

void check_new_mesh(
    wmtk::tests::DEBUG_TriMesh& m,
    std::vector<int> data,
    bool b,
    int vertex_count,
    int edge_count,
    int face_count)
{
    wmtk::tests::DEBUG_TriMesh new_tm = wmtk::components::extract_subset(m, 2, data, b);
    // new_tm.print_vf();
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Vertex) == vertex_count);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Edge) == edge_count);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Face) == face_count);
    // wmtk::ParaviewWriter writer("mesh_smooth", "vertices", new_tm, true, true, true, false);
    // new_tm.serialize(writer);
}

TEST_CASE("2d_tetrahedron_test_case", "[components][extract_subset][2D]")
{
    wmtk::tests::DEBUG_TriMesh tm = wmtk::tests::tetrahedron_with_position();
    for (int i1 = 0; i1 < 2; ++i1) {
        for (int i2 = 0; i2 < 2; ++i2) {
            for (int i3 = 0; i3 < 2; ++i3) {
                for (int i4 = 0; i4 < 2; ++i4) {
                    std::vector<int> tag_vector = {i1, i2, i3, i4};
                    switch (i1 + i2 + i3 + i4) {
                    // TODO: what to return if none of the faces are tagged? NULL?
                    // Maybe construct a trimesh with 0 vertices
                    case 1: check_new_mesh(tm, tag_vector, true, 3, 3, 1); break;
                    case 2: check_new_mesh(tm, tag_vector, true, 4, 5, 2); break;
                    case 3: check_new_mesh(tm, tag_vector, true, 4, 6, 3); break;
                    case 4: check_new_mesh(tm, tag_vector, true, 4, 6, 4); break;
                    }
                }
            }
        }
    }
}

TEST_CASE("2d_9tri_with_a_hole_test_case", "[components][extract_subset][2D]")
{
    wmtk::tests::DEBUG_TriMesh tm = wmtk::tests::nine_triangles_with_a_hole();
    const unsigned long test_size = 500;
    std::vector<int> tag_vector(tm.capacity(wmtk::PrimitiveType::Face), 0);
    for (size_t i = 0; i < test_size; ++i) {
        std::mt19937 mt{};
        std::uniform_int_distribution tag{0, 1};
        for (int j = 0; j < tag_vector.size(); ++j) {
            tag_vector[j] = tag(mt);
        }
        // check_new_mesh(tm, tag_vector, false, 0, 0, 0);
        std::fill(tag_vector.begin(), tag_vector.end(), 0);
    }
}