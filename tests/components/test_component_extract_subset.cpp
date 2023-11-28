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
                        // case 1: check_new_mesh(tm, tag_vector, true, 3, 3, 1); break;
                        // case 2: check_new_mesh(tm, tag_vector, true, 4, 5, 2); break;
                        // case 3: check_new_mesh(tm, tag_vector, true, 4, 6, 3); break;
                        // case 4: check_new_mesh(tm, tag_vector, true, 4, 6, 4); break;
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

TEST_CASE("component_3+4_test_case", "[components][extract_subset][2D][manual]")
{
    wmtk::tests::DEBUG_TriMesh tm;
    wmtk::RowVectors3l tris;
    tris.resize(46, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 0, 2, 3;
    tris.row(2) << 1, 2, 4;
    tris.row(3) << 2, 3, 4;
    tris.row(4) << 0, 3, 5;
    tris.row(5) << 3, 4, 5;
    tris.row(6) << 0, 5, 6;
    tris.row(7) << 5, 6, 7;
    tris.row(8) << 4, 5, 8;
    tris.row(9) << 0, 6, 7;
    tris.row(10) << 5, 7, 8;
    tris.row(11) << 0, 7, 9;
    tris.row(12) << 7, 8, 9;
    tris.row(13) << 0, 9, 10;
    tris.row(14) << 9, 10, 12;
    tris.row(15) << 9, 12, 11;
    tris.row(16) << 9, 8, 11;
    tris.row(17) << 0, 10, 13;
    tris.row(18) << 10, 12, 13;
    tris.row(19) << 13, 12, 15;
    tris.row(20) << 12, 15, 16;
    tris.row(21) << 11, 12, 16;
    tris.row(22) << 11, 16, 17;
    tris.row(23) << 15, 16, 17;
    tris.row(24) << 13, 15, 17;
    tris.row(25) << 0, 13, 14;
    tris.row(26) << 13, 14, 17;
    tris.row(27) << 0, 14, 20;
    tris.row(28) << 14, 18, 20;
    tris.row(29) << 14, 18, 17;
    tris.row(30) << 17, 18, 19;
    tris.row(31) << 0, 20, 24;
    tris.row(32) << 20, 18, 24;
    tris.row(33) << 18, 19, 24;
    tris.row(34) << 19, 21, 24;
    tris.row(35) << 0, 24, 23;
    tris.row(36) << 24, 21, 23;
    tris.row(37) << 21, 22, 23;
    tris.row(38) << 0, 23, 27;
    tris.row(39) << 27, 23, 26;
    tris.row(40) << 23, 22, 26;
    tris.row(41) << 0, 27, 29;
    tris.row(42) << 29, 27, 28;
    tris.row(43) << 28, 27, 26;
    tris.row(44) << 0, 25, 29;
    tris.row(45) << 25, 28, 29;
    tm.initialize(tris);

    std::vector<int> tag_vector(tm.capacity(wmtk::PrimitiveType::Face), 0);
    std::vector<int> id = {0,  1,  2,  3,  5,  6,  7,  8,  10, 11, 12, 25, 26, 29,
                           30, 31, 32, 33, 34, 36, 37, 38, 39, 40, 42, 43, 44, 45};
    for (auto i : id) tag_vector[i] = 1;
    wmtk::tests::DEBUG_TriMesh new_tm = wmtk::components::extract_subset(tm, 2, tag_vector, false);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Vertex) == 25);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Face) == 28);
    // new_tm.print_vf();
    auto topo_tm = wmtk::components::internal::topology_separate_2d(new_tm);
    CHECK(topo_tm.capacity(wmtk::PrimitiveType::Vertex) == 31);
    CHECK(topo_tm.capacity(wmtk::PrimitiveType::Face) == 28);
}