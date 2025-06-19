#include <wmtk/TriMesh.h>

#include <igl/read_triangle_mesh.h>
#include <stdlib.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>

using namespace wmtk;

TEST_CASE("load mesh and create TriMesh", "[test_mesh_creation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    m.init(3, tris);
    REQUIRE(m.tri_capacity() == tris.size());
    REQUIRE(m.vert_capacity() == 3);
}

TEST_CASE("test generate tuples with 1 triangle", "[test_tuple_generation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    m.init(3, tris);

    SECTION("test generation from vertics")
    {
        auto vertices_tuples = m.get_vertices();
        REQUIRE(vertices_tuples.size() == 3);
        REQUIRE(vertices_tuples[0].vid(m) == 0);
        REQUIRE(vertices_tuples[1].vid(m) == 1);
        REQUIRE(vertices_tuples[2].vid(m) == 2);
    }
    SECTION("test generation from faces")
    {
        auto faces_tuples = m.get_faces();
        REQUIRE(faces_tuples.size() == 1);

        // to test vid initialized correctly
        REQUIRE(faces_tuples[0].vid(m) == tris[0][0]);

        // // to test the fid is a triangle touching this vertex
        // std::vector<size_t> tris = m_vertex_connectivity[faces_tuples[0].vid(*this)].m_conn_tris;
        // REQUIRE(std::find(tris.begin(), tris.end(), faces_tuples[0].fid(*this)) != tris.end());
    }

    SECTION("test generation from edges")
    {
        auto edges_tuples = m.get_edges();
        REQUIRE(edges_tuples.size() == 3);
    }
}

TEST_CASE("test generate tuples with 2 triangle", "[test_tuple_generation]")
{
    // 	   v3     /
    //     / \    /
    // 	  /f1 \   /
    // v2 -----v1 /
    // 	  \f0 /   /
    //     \ /    /
    // 	    v0    /
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{2, 1, 3}}};
    m.init(4, tris);

    SECTION("test generation from vertics")
    {
        auto vertices_tuples = m.get_vertices();
        REQUIRE(vertices_tuples.size() == 4);
        REQUIRE(vertices_tuples[0].vid(m) == 0);
        REQUIRE(vertices_tuples[1].vid(m) == 1);
        REQUIRE(vertices_tuples[2].vid(m) == 2);
        REQUIRE(vertices_tuples[3].vid(m) == 3);

        // test the faces are assigned correctly
        REQUIRE(vertices_tuples[1].fid(m) == 0);
        REQUIRE(vertices_tuples[2].fid(m) == 0);
    }

    SECTION("test generation from faces")
    {
        auto faces_tuples = m.get_faces();
        REQUIRE(faces_tuples.size() == 2);

        // std::vector<size_t> conn_tris =
        //     m_vertex_connectivity[faces_tuples[0].vid(*this)].m_conn_tris;
        // REQUIRE(
        //     std::find(conn_tris.begin(), conn_tris.end(), faces_tuples[0].fid(*this)) !=
        //     conn_tris.end());
    }

    SECTION("test generation from edges")
    {
        auto edges_tuples = m.get_edges();
        REQUIRE(edges_tuples.size() == 5);
        REQUIRE(edges_tuples[0].fid(m) == 0);
        REQUIRE(edges_tuples[1].fid(m) == 0);
        REQUIRE(edges_tuples[2].fid(m) == 0);
        REQUIRE(edges_tuples[3].fid(m) == 1);
        REQUIRE(edges_tuples[4].fid(m) == 1);
    }
}

// for every quiry do a require
TEST_CASE("random 10 switches on 2 traingles", "[tuple_operation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 2, 3}}};
    m.init(4, tris);

    SECTION("test all tuples generated using vertices")
    {
        auto vertices_tuples = m.get_vertices();
        for (int i = 0; i < vertices_tuples.size(); i++) {
            TriMesh::Tuple v_tuple = vertices_tuples[i];
            for (int j = 0; j < 10; j++) {
                size_t test = rand() % 3;
                switch (test) {
                case 0: v_tuple = v_tuple.switch_vertex(m); break;
                case 1: v_tuple = v_tuple.switch_edge(m); break;
                case 2: v_tuple = v_tuple.switch_face(m).value_or(v_tuple); break;
                default:;
                }
            }
            REQUIRE(v_tuple.is_valid(m));
        }
    }

    SECTION("test all tuples generated using edges")
    {
        auto edges_tuples = m.get_edges();
        for (int i = 0; i < edges_tuples.size(); i++) {
            TriMesh::Tuple e_tuple = edges_tuples[i];
            for (int j = 0; j < 10; j++) {
                size_t test = rand() % 3;
                switch (test) {
                case 0: e_tuple = e_tuple.switch_vertex(m); break;
                case 1: e_tuple = e_tuple.switch_edge(m); break;
                case 2: e_tuple = e_tuple.switch_face(m).value_or(e_tuple); break;
                default:;
                }
            }
            REQUIRE(e_tuple.is_valid(m));
        }
    }

    SECTION("test all tuples generated using faces")
    {
        auto faces_tuples = m.get_faces();
        for (int i = 0; i < faces_tuples.size(); i++) {
            TriMesh::Tuple f_tuple = faces_tuples[i];
            for (int j = 0; j < 10; j++) {
                size_t test = rand() % 3;
                switch (test) {
                case 0: f_tuple = f_tuple.switch_vertex(m); break;
                case 1: f_tuple = f_tuple.switch_edge(m); break;
                case 2: f_tuple = f_tuple.switch_face(m).value_or(f_tuple); break;
                default:;
                }
            }
            REQUIRE(f_tuple.is_valid(m));
        }
    }
}

TriMesh::Tuple double_switch_vertex(TriMesh::Tuple t, TriMesh& m)
{
    TriMesh::Tuple t_after = t.switch_vertex(m);
    t_after = t_after.switch_vertex(m);
    return t_after;
}

TriMesh::Tuple double_switch_edge(TriMesh::Tuple t, TriMesh& m)
{
    TriMesh::Tuple t_after = t.switch_edge(m);
    t_after = t_after.switch_edge(m);
    return t_after;
}

TriMesh::Tuple double_switch_face(TriMesh::Tuple t, TriMesh& m)
{
    TriMesh::Tuple t_after = t.switch_face(m).value_or(t);
    t_after = t_after.switch_face(m).value_or(t);
    return t_after;
}

bool tuple_equal(const TriMesh& m, TriMesh::Tuple t1, TriMesh::Tuple t2)
{
    return (t1.fid(m) == t2.fid(m)) && (t1.eid(m) == t2.eid(m)) && (t1.vid(m) == t2.vid(m));
}


// checking for every tuple t:
// (1) t.switch_vertex().switch_vertex() == t
// (2) t.switch_edge().switch_edge() == t
// (3) t.switch_tri().switch_tri() == t
TEST_CASE("double switches is identity", "[tuple_operation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 2, 3}}};
    m.init(4, tris);

    SECTION("test all tuples generated using vertices")
    {
        TriMesh::Tuple v_tuple_after;
        auto vertices_tuples = m.get_vertices();
        for (int i = 0; i < vertices_tuples.size(); i++) {
            TriMesh::Tuple v_tuple = vertices_tuples[i];
            v_tuple_after = double_switch_vertex(v_tuple, m);
            REQUIRE(tuple_equal(m, v_tuple_after, v_tuple));
            v_tuple_after = double_switch_edge(v_tuple, m);
            REQUIRE(tuple_equal(m, v_tuple_after, v_tuple));
            v_tuple_after = double_switch_face(v_tuple, m);
            REQUIRE(tuple_equal(m, v_tuple_after, v_tuple));
        }
    }

    SECTION("test all tuples generated using edges")
    {
        TriMesh::Tuple e_tuple_after;
        auto edges_tuples = m.get_edges();
        for (int i = 0; i < edges_tuples.size(); i++) {
            TriMesh::Tuple e_tuple = edges_tuples[i];
            e_tuple_after = double_switch_vertex(e_tuple, m);
            REQUIRE(tuple_equal(m, e_tuple_after, e_tuple));
            e_tuple_after = double_switch_edge(e_tuple, m);
            REQUIRE(tuple_equal(m, e_tuple_after, e_tuple));
            e_tuple_after = double_switch_face(e_tuple, m);
            REQUIRE(tuple_equal(m, e_tuple_after, e_tuple));
        }
    }

    SECTION("test all tuples generated using faces")
    {
        TriMesh::Tuple f_tuple_after;
        auto faces_tuples = m.get_faces();
        for (int i = 0; i < faces_tuples.size(); i++) {
            TriMesh::Tuple f_tuple = faces_tuples[i];
            f_tuple_after = double_switch_vertex(f_tuple, m);
            REQUIRE(tuple_equal(m, f_tuple_after, f_tuple));
            f_tuple_after = double_switch_edge(f_tuple, m);
            REQUIRE(tuple_equal(m, f_tuple_after, f_tuple));
            f_tuple_after = double_switch_face(f_tuple, m);
            REQUIRE(tuple_equal(m, f_tuple_after, f_tuple));
        }
    }
}
// check for every t
// t.switch_vertex().switchedge().switchvertex().switchedge().switchvertex().switchedge() == t
TEST_CASE("vertex_edge switches equals indentity", "[tuple_operation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 2, 3}}};
    m.init(4, tris);

    SECTION("test all tuples generated using vertices")
    {
        TriMesh::Tuple v_tuple_after;
        auto vertices_tuples = m.get_vertices();
        for (int i = 0; i < vertices_tuples.size(); i++) {
            TriMesh::Tuple v_tuple = vertices_tuples[i];
            v_tuple_after = v_tuple.switch_vertex(m);
            v_tuple_after = v_tuple_after.switch_edge(m);
            v_tuple_after = v_tuple_after.switch_vertex(m);
            v_tuple_after = v_tuple_after.switch_edge(m);
            v_tuple_after = v_tuple_after.switch_vertex(m);
            v_tuple_after = v_tuple_after.switch_edge(m);
            REQUIRE(tuple_equal(m, v_tuple, v_tuple_after));
        }
    }

    SECTION("test all tuples generated using edges")
    {
        TriMesh::Tuple e_tuple_after;
        auto edges_tuples = m.get_edges();
        for (int i = 0; i < edges_tuples.size(); i++) {
            TriMesh::Tuple e_tuple = edges_tuples[i];
            e_tuple_after = e_tuple.switch_vertex(m);
            e_tuple_after = e_tuple_after.switch_edge(m);
            e_tuple_after = e_tuple_after.switch_vertex(m);
            e_tuple_after = e_tuple_after.switch_edge(m);
            e_tuple_after = e_tuple_after.switch_vertex(m);
            e_tuple_after = e_tuple_after.switch_edge(m);
            REQUIRE(tuple_equal(m, e_tuple, e_tuple_after));
        }
    }

    SECTION("test all tuples generated using faces")
    {
        TriMesh::Tuple f_tuple_after;
        auto faces_tuples = m.get_faces();
        for (int i = 0; i < faces_tuples.size(); i++) {
            TriMesh::Tuple f_tuple = faces_tuples[i];
            f_tuple_after = f_tuple.switch_vertex(m);
            f_tuple_after = f_tuple_after.switch_edge(m);
            f_tuple_after = f_tuple_after.switch_vertex(m);
            f_tuple_after = f_tuple_after.switch_edge(m);
            f_tuple_after = f_tuple_after.switch_vertex(m);
            f_tuple_after = f_tuple_after.switch_edge(m);
            REQUIRE(tuple_equal(m, f_tuple, f_tuple_after));
        }
    }
}

TEST_CASE("test_link_check", "[test_pre_check]")
{
    TriMesh m;
    SECTION("extra_face_after_collapse")
    {
        std::vector<std::array<size_t, 3>> tris =
            {{{1, 2, 3}}, {{0, 1, 4}}, {{0, 2, 5}}, {{0, 1, 6}}, {{0, 2, 6}}, {{1, 2, 6}}};
        m.init(7, tris);
        TriMesh::Tuple edge(1, 2, 0, m);
        REQUIRE_FALSE(m.check_link_condition(edge));
    }
    SECTION("one_triangle")
    {
        std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
        m.init(3, tris);

        TriMesh::Tuple edge(0, 2, 0, m);
        assert(edge.is_valid(m));
        REQUIRE_FALSE(m.check_link_condition(edge));
    }
    SECTION("one_tet")
    {
        std::vector<std::array<size_t, 3>> tris = {
            {{0, 1, 2}},
            {{1, 3, 2}},
            {{0, 2, 3}},
            {{3, 0, 1}}};
        m.init(4, tris);

        TriMesh::Tuple edge(1, 0, 0, m);
        assert(edge.is_valid(m));
        REQUIRE_FALSE(m.check_link_condition(edge));
    }
    SECTION("non_manifold_after_collapse")
    {
        std::vector<std::array<size_t, 3>> tris = {
            {{0, 1, 5}},
            {{1, 2, 5}},
            {{2, 3, 5}},
            {{5, 3, 4}}};
        m.init(6, tris);

        TriMesh::Tuple fail_edge(5, 0, 1, m);
        REQUIRE_FALSE(m.check_link_condition(fail_edge));
        TriMesh::Tuple pass_edge(0, 2, 0, m);
        REQUIRE(m.check_link_condition(pass_edge));
    }
}
// test manifold (eid uniqueness)
TEST_CASE("test unique edge", "[test_2d_operation]")
{
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 3, 2}}, {{4, 1, 0}}, {{0, 2, 5}}};
    auto m = TriMesh();
    m.init(6, tris);
    REQUIRE(m.check_edge_manifold());
}

TEST_CASE("edge_collapse", "[test_2d_operation]")
{
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 3, 2}}, {{4, 1, 0}}, {{0, 2, 5}}};
    SECTION("rollback")
    {
        class NoCollapseMesh : public TriMesh
        {
            bool collapse_edge_before(const TriMesh::Tuple& loc) override { return true; };
            bool collapse_edge_after(const TriMesh::Tuple& loc) override { return false; };
        };
        auto m = NoCollapseMesh();
        m.init(6, tris);
        const auto tuple = NoCollapseMesh::Tuple(1, 0, 0, m);
        REQUIRE(tuple.is_valid(m));
        std::vector<TriMesh::Tuple> dummy;
        REQUIRE_FALSE(m.collapse_edge(tuple, dummy));
        REQUIRE(tuple.is_valid(m));
    }
    SECTION("collapse")
    {
        class Collapse : public TriMesh
        {
            bool collapse_edge_before(const TriMesh::Tuple& loc) override { return true; };
            bool collapse_edge_after(const TriMesh::Tuple& loc) override { return true; };
        };
        auto m = Collapse();

        m.init(6, tris);
        const auto tuple = Collapse::Tuple(1, 0, 0, m);

        REQUIRE(tuple.is_valid(m));
        std::vector<TriMesh::Tuple> dummy;

        REQUIRE(m.collapse_edge(tuple, dummy)); // fail at check manifold
        REQUIRE_FALSE(tuple.is_valid(m));
    }
}

TEST_CASE("swap_operation", "[test_2d_operation]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/fan.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    REQUIRE(ok);
    std::vector<std::array<size_t, 3>> tri(F.rows());

    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    TriMesh m;
    m.init(V.rows(), tri);

    SECTION("swap")
    {
        TriMesh::Tuple swap_e = TriMesh::Tuple(0, 0, 0, m);
        REQUIRE(swap_e.is_valid(m));
        std::vector<TriMesh::Tuple> new_e;
        REQUIRE(m.swap_edge(swap_e, new_e));
        REQUIRE_FALSE(swap_e.is_valid(m));
        REQUIRE(new_e.front().is_valid(m));
        REQUIRE(m.vert_capacity() == 8);
        REQUIRE(m.tri_capacity() == 8);

        std::vector<size_t> m_indices;
        for (const auto& edge : m.get_one_ring_edges_for_vertex(new_e.front())) {
            m_indices.push_back(edge.vid(m));
        }
        vector_unique(m_indices);
        std::vector<size_t> tru_indices = {0, 2, 3, 4, 5};
        REQUIRE(std::equal(m_indices.begin(), m_indices.end(), tru_indices.begin()));
    }
    SECTION("swap_on_boundary_edge")
    {
        TriMesh::Tuple swap_e = TriMesh::Tuple(0, 2, 0, m);
        REQUIRE(swap_e.is_valid(m));
        std::vector<TriMesh::Tuple> dummy;
        REQUIRE_FALSE(m.swap_edge(swap_e, dummy));
    }
    SECTION("swap_on_connected_vertices")
    {
        TriMesh m2;
        std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 0, 3}}, {{1, 3, 2}}};
        m2.init(4, tris);
        TriMesh::Tuple edge(0, 2, 0, m);
        assert(edge.is_valid(m));
        std::vector<TriMesh::Tuple> dummy;
        REQUIRE_FALSE(m.swap_edge(edge, dummy));
    }
}

TEST_CASE("split_operation", "[test_2d_operation]")
{
    TriMesh m;
    SECTION("1_tri_split")
    {
        std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
        m.init(3, tris);
        auto edges = m.get_edges();
        TriMesh::Tuple edge(0, 1, 0, m);
        std::vector<TriMesh::Tuple> dummy;
        assert(edge.is_valid(m));
        REQUIRE(m.split_edge(edge, dummy));
        REQUIRE_FALSE(edges[0].is_valid(m));
    }
    SECTION("2_tris_split")
    {
        std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 3, 2}}};
        m.init(4, tris);
        auto edges = m.get_edges();
        TriMesh::Tuple edge(1, 0, 0, m);
        std::vector<TriMesh::Tuple> dummy;
        assert(edge.is_valid(m));
        REQUIRE(m.split_edge(edge, dummy));
        for (auto e : edges) REQUIRE_FALSE(e.is_valid(m));
    }
}
