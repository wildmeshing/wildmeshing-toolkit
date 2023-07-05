#include <stdlib.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
//#include <igl/read_triangle_mesh.h>
#include <wmtk/utils/trimesh_topology_initialization.h>
#include <wmtk/TriMesh.hpp>

using namespace wmtk;

TEST_CASE("2D_initialize", "[test_mesh_creation],[test_tuple_2d]")
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};

    SECTION("init with FV, FE, FF, VF, EF")
    {
        const auto [FE, FF, VF, EF] = trimesh_topology_initialization(tris);

        m.initialize(tris, FE, FF, VF, EF);
    }
    SECTION("init directly from RowVectors3l")
    {
        m.initialize(tris);
    }

    const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
    REQUIRE(vertices.size() == 3);
    return;
    const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
    REQUIRE(edges.size() == 3);
    const std::vector<Tuple> faces = m.get_all(PrimitiveType::Face);
    REQUIRE(faces.size() == 1);


    REQUIRE(m.is_connectivity_valid());

    const Tuple t = m.get_all(PrimitiveType::Face)[0];
    REQUIRE(m.is_valid(t));
}

TEST_CASE("2D_1_triangle", "[test_tuple_generation],[test_tuple_2d]")
{
    TriMesh m;
    {
        RowVectors3l tris;
        tris.resize(1, 3);
        tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
        m.initialize(tris);
    }

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 3);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m._debug_id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m._debug_id(vertices[2], PrimitiveType::Vertex) == 2);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Face) == 0);
        CHECK(m._debug_id(vertices[1], PrimitiveType::Face) == 0);
        CHECK(m._debug_id(vertices[2], PrimitiveType::Face) == 0);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 3);
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Face);
        REQUIRE(faces.size() == 1);
        CHECK(m._debug_id(faces[0], PrimitiveType::Face) == 0);
    }
}

TEST_CASE("2D_2_triangles", "[test_tuple_generation],[test_tuple_2d]")
{
    // 	   v3     /
    //     / \    /
    // 	  /f1 \   /
    // v2 -----v1 /
    // 	  \f0 /   /
    //     \ /    /
    // 	    v0    /

    TriMesh m;
    {
        RowVectors3l tris;
        tris.resize(2, 3);
        tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
        tris.row(1) = Eigen::Matrix<long, 3, 1>{2, 1, 3};
        m.initialize(tris);
    }

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 4);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m._debug_id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m._debug_id(vertices[2], PrimitiveType::Vertex) == 2);
        CHECK(m._debug_id(vertices[3], PrimitiveType::Vertex) == 3);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Face) == 0);
        CHECK(m._debug_id(vertices[3], PrimitiveType::Face) == 1);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 5);
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Face);
        REQUIRE(faces.size() == 2);
        CHECK(m._debug_id(faces[0], PrimitiveType::Face) == 0);
        CHECK(m._debug_id(faces[1], PrimitiveType::Face) == 1);
    }
}

// for every quiry do a require
TEST_CASE("2D_random_switches", "[tuple_operation],[test_tuple_2d]")
{
    // 	   v3     /
    //     / \    /
    // 	  /f1 \   /
    // v2 -----v1 /
    // 	  \f0 /   /
    //     \ /    /
    // 	    v0    /

    TriMesh m;
    {
        RowVectors3l tris;
        tris.resize(2, 3);
        tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
        tris.row(1) = Eigen::Matrix<long, 3, 1>{2, 1, 3};
        m.initialize(tris);
    }

    SECTION("vertices")
    {
        const std::vector<Tuple> vertex_tuples = m.get_all(PrimitiveType::Vertex);
        for (size_t i = 0; i < vertex_tuples.size(); ++i) {
            Tuple t = vertex_tuples[i];
            for (size_t j = 0; j < 10; j++) {
                switch (rand() % 3) {
                case 0: t = m.switch_tuple(t, PrimitiveType::Vertex); break;
                case 1: t = m.switch_tuple(t, PrimitiveType::Edge); break;
                case 2:
                    if (!m.is_boundary(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Face);
                    }
                    break;
                default: break;
                }
                CHECK(m.is_valid(t));
            }
        }
    }

    SECTION("edges")
    {
        const std::vector<Tuple> edge_tuples = m.get_all(PrimitiveType::Edge);
        for (size_t i = 0; i < edge_tuples.size(); ++i) {
            Tuple t = edge_tuples[i];
            for (size_t j = 0; j < 10; j++) {
                switch (rand() % 3) {
                case 0: t = m.switch_tuple(t, PrimitiveType::Vertex); break;
                case 1: t = m.switch_tuple(t, PrimitiveType::Edge); break;
                case 2:
                    if (!m.is_boundary(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Face);
                    }
                    break;
                default: break;
                }
                CHECK(m.is_valid(t));
            }
        }
    }

    SECTION("faces")
    {
        const std::vector<Tuple> face_tuples = m.get_all(PrimitiveType::Face);
        for (size_t i = 0; i < face_tuples.size(); ++i) {
            Tuple t = face_tuples[i];
            for (size_t j = 0; j < 10; j++) {
                switch (rand() % 3) {
                case 0: t = m.switch_tuple(t, PrimitiveType::Vertex); break;
                case 1: t = m.switch_tuple(t, PrimitiveType::Edge); break;
                case 2:
                    if (!m.is_boundary(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Face);
                    }
                    break;
                default: break;
                }
                CHECK(m.is_valid(t));
            }
        }
    }
}

Tuple double_switch_vertex(const TriMesh& m, const Tuple& t)
{
    Tuple t_after = m.switch_tuple(t, PrimitiveType::Vertex);
    t_after = m.switch_tuple(t_after, PrimitiveType::Vertex);
    return t_after;
}

Tuple double_switch_edge(const TriMesh& m, const Tuple& t)
{
    Tuple t_after = m.switch_tuple(t, PrimitiveType::Edge);
    t_after = m.switch_tuple(t_after, PrimitiveType::Edge);
    return t_after;
}

Tuple double_switch_face(const TriMesh& m, const Tuple& t)
{
    if (m.is_boundary(t)) return t;
    Tuple t_after = m.switch_tuple(t, PrimitiveType::Face);
    t_after = m.switch_tuple(t_after, PrimitiveType::Face);
    return t_after;
}

bool tuple_equal(const TriMesh& m, const Tuple& t0, const Tuple& t1)
{
    const long v0 = m._debug_id(t0, PrimitiveType::Vertex);
    const long e0 = m._debug_id(t0, PrimitiveType::Edge);
    const long f0 = m._debug_id(t0, PrimitiveType::Face);
    const long v1 = m._debug_id(t1, PrimitiveType::Vertex);
    const long e1 = m._debug_id(t1, PrimitiveType::Edge);
    const long f1 = m._debug_id(t1, PrimitiveType::Face);
    return (v0 == v1) && (e0 == e1) && (f0 == f1);
}


// // checking for every tuple t:
// // (1) t.switch_vertex().switch_vertex() == t
// // (2) t.switch_edge().switch_edge() == t
// // (3) t.switch_tri().switch_tri() == t
TEST_CASE("2D_double_switches", "[tuple_operation],[test_tuple_2d]")
{
    // 	   v3     /
    //     / \    /
    // 	  /f1 \   /
    // v2 -----v1 /
    // 	  \f0 /   /
    //     \ /    /
    // 	    v0    /

    TriMesh m;
    {
        RowVectors3l tris;
        tris.resize(2, 3);
        tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
        tris.row(1) = Eigen::Matrix<long, 3, 1>{2, 1, 3};
        m.initialize(tris);
    }

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 4);
        for (const auto& t : vertices) {
            const Tuple t_after_v = double_switch_vertex(m, t);
            CHECK(tuple_equal(m, t, t_after_v));
            const Tuple t_after_e = double_switch_edge(m, t);
            CHECK(tuple_equal(m, t, t_after_e));
            const Tuple t_after_f = double_switch_face(m, t);
            CHECK(tuple_equal(m, t, t_after_f));
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 5);
        for (const auto& t : edges) {
            const Tuple t_after_v = double_switch_vertex(m, t);
            CHECK(tuple_equal(m, t, t_after_v));
            const Tuple t_after_e = double_switch_edge(m, t);
            CHECK(tuple_equal(m, t, t_after_e));
            const Tuple t_after_f = double_switch_face(m, t);
            CHECK(tuple_equal(m, t, t_after_f));
        }
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Face);
        REQUIRE(faces.size() == 2);
        for (const auto& t : faces) {
            const Tuple t_after_v = double_switch_vertex(m, t);
            CHECK(tuple_equal(m, t, t_after_v));
            const Tuple t_after_e = double_switch_edge(m, t);
            CHECK(tuple_equal(m, t, t_after_e));
            const Tuple t_after_f = double_switch_face(m, t);
            CHECK(tuple_equal(m, t, t_after_f));
        }
    }
}


// // check for every t
// // t.switch_vertex().switchedge().switchvertex().switchedge().switchvertex().switchedge() == t
TEST_CASE("2D_vertex_edge_switches", "[tuple_operation],[test_tuple_2d]")
{
    TriMesh m;
    {
        RowVectors3l tris;
        tris.resize(2, 3);
        tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
        tris.row(1) = Eigen::Matrix<long, 3, 1>{2, 1, 3};
        m.initialize(tris);
    }

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 4);
        for (const auto& t : vertices) {
            Tuple t_iter = t;
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Vertex);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Vertex);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Vertex);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            CHECK(tuple_equal(m, t, t_iter));
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 5);
        for (const auto& t : edges) {
            Tuple t_iter = t;
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Vertex);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Vertex);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Vertex);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            CHECK(tuple_equal(m, t, t_iter));
        }
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Face);
        REQUIRE(faces.size() == 2);
        for (const auto& t : faces) {
            Tuple t_iter = t;
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Vertex);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Vertex);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Vertex);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            CHECK(tuple_equal(m, t, t_iter));
        }
    }
}

TEST_CASE("2D_one_ring_iteration", "[tuple_operation],[test_tuple_2d]")
{
    TriMesh m;
    {
        RowVectors3l tris;
        tris.resize(6, 3);
        tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
        tris.row(1) = Eigen::Matrix<long, 3, 1>{0, 2, 3};
        tris.row(2) = Eigen::Matrix<long, 3, 1>{0, 3, 4};
        tris.row(3) = Eigen::Matrix<long, 3, 1>{0, 4, 5};
        tris.row(4) = Eigen::Matrix<long, 3, 1>{0, 5, 6};
        tris.row(5) = Eigen::Matrix<long, 3, 1>{0, 6, 1};
        m.initialize(tris);
    }

    const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
    REQUIRE(vertices.size() == 7);
    for (const auto& t : vertices) {
        // face-edge switch
        Tuple t_iter = t;
        for (size_t i = 0; i < 6; ++i) {
            if (m.is_boundary(t_iter)) {
                break;
            }
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Face);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            if (tuple_equal(m, t, t_iter)) {
                break;
            }
        }
        CHECK((tuple_equal(m, t, t_iter) || m.is_boundary(t_iter)));
        // edge-face switch
        t_iter = t;
        for (size_t i = 0; i < 6; ++i) {
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            if (m.is_boundary(t_iter)) {
                break;
            }
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Face);
            if (tuple_equal(m, t, t_iter)) {
                break;
            }
        }
        CHECK((tuple_equal(m, t, t_iter) || m.is_boundary(t_iter)));
    }
}


// TEST_CASE("test_link_check", "[test_pre_check]")
//{
// TriMesh m;
//{
//     RowVectors3l tris;
//     tris.resize(6, 3);
//     tris.row(0) = Eigen::Matrix<long, 3, 1>{1, 2, 3};
//     tris.row(1) = Eigen::Matrix<long, 3, 1>{0, 1, 4};
//     tris.row(2) = Eigen::Matrix<long, 3, 1>{0, 2, 5};
//     tris.row(3) = Eigen::Matrix<long, 3, 1>{0, 1, 6};
//     tris.row(4) = Eigen::Matrix<long, 3, 1>{0, 2, 6};
//     tris.row(5) = Eigen::Matrix<long, 3, 1>{1, 2, 6};
//     m.initialize(tris);
// }
//      TriMesh m;
//      SECTION("extra_face_after_collapse")
//      {
//          std::vector<std::array<size_t, 3>> tris =
//              {{{1, 2, 3}}, {{0, 1, 4}}, {{0, 2, 5}}, {{0, 1, 6}}, {{0, 2, 6}}, {{1, 2, 6}}};
//          m.create_mesh(7, tris);
//          TriMesh::Tuple edge(1, 2, 0, m);

//         REQUIRE(edge.vid(m) == 1);
//         REQUIRE(edge.switch_vertex(m).vid(m) == 2);

//         {
//             // short test for tris_bounded_by_edge
//             std::vector<TriMeshTuple> faces = m.tris_bounded_by_edge(edge);
//             for (const auto& f : faces) {
//                 REQUIRE(f.is_valid(m));
//             }
//             std::vector<size_t> fids;
//             std::transform(
//                 faces.begin(),
//                 faces.end(),
//                 std::back_inserter(fids),
//                 [&](const TriMeshTuple& t) -> size_t { return t.fid(m); });
//             REQUIRE(std::is_sorted(fids.begin(), fids.end()));
//             CHECK(fids[0] == 0);
//             CHECK(fids[1] == 5);
//         }

//         REQUIRE_FALSE(TriMeshEdgeCollapseOperation::check_link_condition(m, edge));
//     }
//     SECTION("one_triangle")
//     {
//         std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
//         m.create_mesh(3, tris);

//         TriMesh::Tuple edge(0, 2, 0, m);
//         assert(edge.is_valid(m));
//         REQUIRE_FALSE(TriMeshEdgeCollapseOperation::check_link_condition(m, edge));
//     }
//     SECTION("one_tet")
//     {
//         std::vector<std::array<size_t, 3>> tris = {
//             {{0, 1, 2}},
//             {{1, 3, 2}},
//             {{0, 2, 3}},
//             {{3, 0, 1}}};
//         m.create_mesh(4, tris);

//         TriMesh::Tuple edge(1, 0, 0, m);
//         assert(edge.is_valid(m));
//         REQUIRE_FALSE(TriMeshEdgeCollapseOperation::check_link_condition(m, edge));
//     }
//     SECTION("non_manifold_after_collapse")
//     {
//         std::vector<std::array<size_t, 3>> tris = {
//             {{0, 1, 5}},
//             {{1, 2, 5}},
//             {{2, 3, 5}},
//             {{5, 3, 4}}};
//         m.create_mesh(6, tris);

//         TriMesh::Tuple fail_edge(5, 0, 1, m);
//         REQUIRE_FALSE(TriMeshEdgeCollapseOperation::check_link_condition(m, fail_edge));
//         TriMesh::Tuple pass_edge(0, 2, 0, m);
//         REQUIRE(TriMeshEdgeCollapseOperation::check_link_condition(m, pass_edge));
//     }
//}
// // test manifold (eid uniqueness)
// TEST_CASE("test unique edge", "[test_2d_operation]")
// {
//     std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 3, 2}}, {{4, 1, 0}}, {{0, 2,
//     5}}}; auto m = TriMesh(); m.create_mesh(6, tris); REQUIRE(m.check_edge_manifold());
// }

// TEST_CASE("edge_collapse", "[test_2d_operation]")
// {
//     std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 3, 2}}, {{4, 1, 0}}, {{0, 2,
//     5}}}; SECTION("rollback")
//     {
//         class NoCollapseCollapseOperation : public wmtk::TriMeshEdgeCollapseOperation
//         {
//             bool before(TriMesh&, const TriMesh::Tuple&) override { return true; };
//             bool after(TriMesh&) override { return false; };
//         } collapse_op;
//         TriMesh m;
//         // auto m = NoCollapseMesh();
//         m.create_mesh(6, tris);
//         const auto tuple = TriMesh::Tuple(1, 0, 0, m);
//         REQUIRE(tuple.is_valid(m));
//         std::vector<TriMesh::Tuple> dummy;
//         REQUIRE_FALSE(collapse_op(m, tuple));
//         REQUIRE(tuple.is_valid(m));
//     }
//     SECTION("collapse")
//     {
//         class AllCollapseCollapseOperation : public wmtk::TriMeshEdgeCollapseOperation
//         {
//             bool before(TriMesh&, const TriMesh::Tuple&) override { return true; };
//             bool after(TriMesh&) override { return true; };
//         } collapse_op;
//         TriMesh m;

//         m.create_mesh(6, tris);
//         const auto tuple = TriMesh::Tuple(1, 0, 0, m);

//         REQUIRE(tuple.is_valid(m));

//         REQUIRE(collapse_op(m, tuple)); // fail at check manifold
//         REQUIRE_FALSE(tuple.is_valid(m));
//     }
// }

// TEST_CASE("swap_operation", "[test_2d_operation]")
// {
//     wmtk::TriMeshEdgeSwapOperation swap_op;
//     SECTION("swap")
//     {
//         TriMesh m;
//         std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{3, 1, 0}}};
//         m.create_mesh(4, tris);
//         TriMesh::Tuple edge(0, 2, 0, m);
//         assert(edge.is_valid(m));

//         REQUIRE(swap_op(m, edge));
//     }
//     SECTION("swap_boundary")
//     {
//         TriMesh m;
//         std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{3, 1, 0}}};
//         m.create_mesh(4, tris);
//         TriMesh::Tuple edge(0, 1, 0, m);
//         assert(edge.is_valid(m));

//         REQUIRE_FALSE(swap_op(m, edge));
//     }

//     SECTION("swap_on_connected_vertices")
//     {
//         TriMesh m2;
//         std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 0, 3}}, {{1, 3, 2}}};
//         m2.create_mesh(4, tris);
//         TriMesh::Tuple edge(0, 2, 0, m2);
//         assert(edge.is_valid(m2));
//         REQUIRE_FALSE(swap_op(m2, edge));
//     }

//     SECTION("swap 4 times retain start tuple")
//     {
//         TriMesh m3;
//         std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{3, 1, 0}}};
//         m3.create_mesh(4, tris);
//         TriMesh::Tuple edge(0, 2, 0, m3);
//         assert(edge.is_valid(m3));


//         bool success;

//         success = swap_op(m3, edge);
//         REQUIRE(success);
//         auto new_t = swap_op.modified_triangles(m3)[0];
//         assert(new_t.is_valid(m3));

//         success = swap_op(m3, new_t);
//         REQUIRE(success);
//         new_t = swap_op.modified_triangles(m3)[0];
//         assert(new_t.is_valid(m3));

//         success = swap_op(m3, new_t);
//         REQUIRE(success);
//         new_t = swap_op.modified_triangles(m3)[0];
//         assert(new_t.is_valid(m3));

//         success = swap_op(m3, new_t);
//         REQUIRE(success);
//         new_t = swap_op.modified_triangles(m3)[0];
//         REQUIRE(new_t.vid(m3) == 0);
//         REQUIRE(new_t.switch_vertex(m3).vid(m3) == 1);
//         REQUIRE(new_t.switch_edge(m3).switch_vertex(m3).vid(m3) == 2);
//     }
// }

// TEST_CASE("split_operation", "[test_2d_operation]")
// {
//     wmtk::TriMeshEdgeSplitOperation split_op;
//     TriMesh m;
//     SECTION("1_tri_split")
//     {
//         std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
//         m.create_mesh(3, tris);
//         auto edges = m.get_edges();
//         TriMesh::Tuple edge(0, 1, 0, m);
//         assert(edge.is_valid(m));
//         REQUIRE(split_op(m, edge));
//         REQUIRE_FALSE(edges[0].is_valid(m));
//     }
//     SECTION("2_tris_split")
//     {
//         std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 3, 2}}};
//         m.create_mesh(4, tris);
//         auto edges = m.get_edges();
//         TriMesh::Tuple edge(1, 0, 0, m);
//         assert(edge.is_valid(m));
//         REQUIRE(split_op(m, edge));
//         for (auto e : edges) REQUIRE_FALSE(e.is_valid(m));
//     }
// }
