#include <wmtk/TriMesh.h>

#include <stdlib.h>
#include <catch2/catch.hpp>
#include <iostream>

using namespace wmtk;

TEST_CASE("load mesh from file and create TriMesh", "[test_mesh_creation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    m.create_mesh(3, tris);
    REQUIRE(m.get_tri_connectivity().size() == tris.size());
    REQUIRE(m.get_vertex_connectivity().size() == 3);
}

TEST_CASE("test generate tuples with 1 triangle", "[test_tuple_generation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    m.create_mesh(3, tris);
    std::vector<TriMesh::TriangleConnectivity> m_tri_connectivity = m.get_tri_connectivity();
    std::vector<TriMesh::VertexConnectivity> m_vertex_connectivity = m.get_vertex_connectivity();
    SECTION("test generation from vertics")
    {
        auto vertices_tuples = m.generate_tuples_from_vertices();
        REQUIRE(vertices_tuples.size() == 3);
        REQUIRE(vertices_tuples[0].get_vid() == 0);
        REQUIRE(vertices_tuples[1].get_vid() == 1);
        REQUIRE(vertices_tuples[2].get_vid() == 2);
    }
    SECTION("test generation from faces")
    {
        auto faces_tuples = m.generate_tuples_from_faces();
        REQUIRE(faces_tuples.size() == 1);

        // to test vid initialized correctly
        REQUIRE(faces_tuples[0].get_vid() == m_tri_connectivity[0].m_indices[0]);

        // to test the fid is a triangle touching this vertex
        std::vector<size_t> tris = m_vertex_connectivity[faces_tuples[0].get_vid()].m_conn_tris;
        REQUIRE(std::find(tris.begin(), tris.end(), faces_tuples[0].get_fid()) != tris.end());
    }

    SECTION("test generation from edges")
    {
        auto edges_tuples = m.generate_tuples_from_edges();
        REQUIRE(edges_tuples.size() == 3);
    }
}

TEST_CASE("test generate tuples with 2 triangle", "[test_tuple_generation]")
{
    // 	   v3
    //     / \	
    // 	  /f1 \ 
    // v2 -----v1
    // 	  \f0 /
    //     \ /
    // 	    v0
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}, {1, 2, 3}}};
    m.create_mesh(4, tris);

    std::vector<TriMesh::TriangleConnectivity> m_tri_connectivity = m.get_tri_connectivity();
    std::vector<TriMesh::VertexConnectivity> m_vertex_connectivity = m.get_vertex_connectivity();

    SECTION("test generation from vertics")
    {
        auto vertices_tuples = m.generate_tuples_from_vertices();
        REQUIRE(vertices_tuples.size() == 4);
        REQUIRE(vertices_tuples[0].get_vid() == 0);
        REQUIRE(vertices_tuples[1].get_vid() == 1);
        REQUIRE(vertices_tuples[2].get_vid() == 2);
        REQUIRE(vertices_tuples[3].get_vid() == 3);

        // test the faces are assigned correctly
        REQUIRE(vertices_tuples[1].get_fid() == 0);
        REQUIRE(vertices_tuples[2].get_fid() == 0);
    }

    SECTION("test generation from faces")
    {
        auto faces_tuples = m.generate_tuples_from_faces();
        REQUIRE(faces_tuples.size() == 2);

        std::vector<size_t> conn_tris =
            m_vertex_connectivity[faces_tuples[0].get_vid()].m_conn_tris;
        REQUIRE(
            std::find(conn_tris.begin(), conn_tris.end(), faces_tuples[0].get_fid()) !=
            conn_tris.end());
    }

    SECTION("test generation from edges")
    {
        auto edges_tuples = m.generate_tuples_from_edges();
        REQUIRE(edges_tuples.size() == 5);
        REQUIRE(edges_tuples[0].get_fid() == 0);
        REQUIRE(edges_tuples[1].get_fid() == 0);
        REQUIRE(edges_tuples[2].get_fid() == 0);
        REQUIRE(edges_tuples[3].get_fid() == 1);
        REQUIRE(edges_tuples[4].get_fid() == 1);
    }
}

// for every quiry do a require
TEST_CASE("random 10 switches on 2 traingles", "[test_operation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}, {1, 2, 3}}};
    m.create_mesh(4, tris);

    SECTION("test all tuples generated using vertices")
    {
        auto vertices_tuples = m.generate_tuples_from_vertices();
        for (int i = 0; i < vertices_tuples.size(); i++) {
            TriMesh::Tuple v_tuple = vertices_tuples[i];
            for (int j = 0; j < 10; j++) {
                size_t test = rand() % 3;
                switch (test) {
                case 0: v_tuple = v_tuple.switch_vertex(m);
                case 1: v_tuple = v_tuple.switch_edge(m);
                case 2: v_tuple = v_tuple.switch_face(m).value_or(v_tuple);
                }
            }
            REQUIRE(v_tuple.is_valid(m));
        }
    }

    SECTION("test all tuples generated using edges")
    {
        auto edges_tuples = m.generate_tuples_from_edges();
        for (int i = 0; i < edges_tuples.size(); i++) {
            TriMesh::Tuple e_tuple = edges_tuples[i];
            for (int j = 0; j < 10; j++) {
                size_t test = rand() % 3;
                switch (test) {
                case 0: e_tuple = e_tuple.switch_vertex(m);
                case 1: e_tuple = e_tuple.switch_edge(m);
                case 2: e_tuple = e_tuple.switch_face(m).value_or(e_tuple);
                }
            }
            REQUIRE(e_tuple.is_valid(m));
        }
    }

    SECTION("test all tuples generated using faces")
    {
        auto faces_tuples = m.generate_tuples_from_faces();
        for (int i = 0; i < faces_tuples.size(); i++) {
            TriMesh::Tuple f_tuple = faces_tuples[i];
            for (int j = 0; j < 10; j++) {
                size_t test = rand() % 3;
                switch (test) {
                case 0: f_tuple = f_tuple.switch_vertex(m);
                case 1: f_tuple = f_tuple.switch_edge(m);
                case 2: f_tuple = f_tuple.switch_face(m).value_or(f_tuple);
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

bool tuple_equal(TriMesh::Tuple t1, TriMesh::Tuple t2)
{
    if (t1.get_vid() != t2.get_vid())
        std::cout << "vids : " << t1.get_vid() << " " << t2.get_vid() << std::endl;
    if (t1.get_eid() != t2.get_eid())
        std::cout << "eids : " << t1.get_eid() << " " << t2.get_eid() << std::endl;
    if (t1.get_fid() != t2.get_fid())
        std::cout << "fids : " << t1.get_fid() << " " << t2.get_fid() << std::endl;

    return (t1.get_fid() == t2.get_fid()) && (t1.get_eid() == t2.get_eid()) &&
           (t1.get_vid() == t2.get_vid());
}


// checking for every tuple t:
// (1) t.switch_vertex().switch_vertex() == t
// (2) t.switch_edge().switch_edge() == t
// (3) t.switch_tri().switch_tri() == t
TEST_CASE("double switches is identity", "[test_operation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}, {1, 2, 3}}};
    m.create_mesh(4, tris);

    SECTION("test all tuples generated using vertices")
    {
        TriMesh::Tuple v_tuple_after;
        auto vertices_tuples = m.generate_tuples_from_vertices();
        for (int i = 0; i < vertices_tuples.size(); i++) {
            TriMesh::Tuple v_tuple = vertices_tuples[i];
            v_tuple_after = double_switch_vertex(v_tuple, m);
            REQUIRE(tuple_equal(v_tuple_after, v_tuple));
            v_tuple_after = double_switch_edge(v_tuple, m);
            REQUIRE(tuple_equal(v_tuple_after, v_tuple));
            v_tuple_after = double_switch_face(v_tuple, m);
            REQUIRE(tuple_equal(v_tuple_after, v_tuple));
        }
    }

    SECTION("test all tuples generated using edges")
    {
        TriMesh::Tuple e_tuple_after;
        auto edges_tuples = m.generate_tuples_from_edges();
        for (int i = 0; i < edges_tuples.size(); i++) {
            TriMesh::Tuple e_tuple = edges_tuples[i];
            e_tuple_after = double_switch_vertex(e_tuple, m);
            REQUIRE(tuple_equal(e_tuple_after, e_tuple));
            e_tuple_after = double_switch_edge(e_tuple, m);
            REQUIRE(tuple_equal(e_tuple_after, e_tuple));
            e_tuple_after = double_switch_face(e_tuple, m);
            REQUIRE(tuple_equal(e_tuple_after, e_tuple));
        }
    }

    SECTION("test all tuples generated using faces")
    {
        TriMesh::Tuple f_tuple_after;
        auto faces_tuples = m.generate_tuples_from_faces();
        for (int i = 0; i < faces_tuples.size(); i++) {
            TriMesh::Tuple f_tuple = faces_tuples[i];
            f_tuple_after = double_switch_vertex(f_tuple, m);
            REQUIRE(tuple_equal(f_tuple_after, f_tuple));
            f_tuple_after = double_switch_edge(f_tuple, m);
            REQUIRE(tuple_equal(f_tuple_after, f_tuple));
            f_tuple_after = double_switch_face(f_tuple, m);
            REQUIRE(tuple_equal(f_tuple_after, f_tuple));
        }
    }
}
// check for every t
// t.switch_vertex().switchedge().switchvertex().switchedge().switchvertex().switchedge() == t
TEST_CASE("vertex_edge switches equals indentity", "[test_operation]")
{
    TriMesh m;
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}, {1, 2, 3}}};
    m.create_mesh(4, tris);

    SECTION("test all tuples generated using vertices")
    {
        TriMesh::Tuple v_tuple_after;
        auto vertices_tuples = m.generate_tuples_from_vertices();
        for (int i = 0; i < vertices_tuples.size(); i++) {
            TriMesh::Tuple v_tuple = vertices_tuples[i];
            v_tuple_after = v_tuple.switch_vertex(m);
            v_tuple_after = v_tuple_after.switch_edge(m);
            v_tuple_after = v_tuple_after.switch_vertex(m);
            v_tuple_after = v_tuple_after.switch_edge(m);
            v_tuple_after = v_tuple_after.switch_vertex(m);
            v_tuple_after = v_tuple_after.switch_edge(m);
            REQUIRE(tuple_equal(v_tuple, v_tuple_after));
        }
    }

    SECTION("test all tuples generated using edges")
    {
        TriMesh::Tuple e_tuple_after;
        auto edges_tuples = m.generate_tuples_from_edges();
        for (int i = 0; i < edges_tuples.size(); i++) {
            TriMesh::Tuple e_tuple = edges_tuples[i];
            e_tuple_after = e_tuple.switch_vertex(m);
            e_tuple_after = e_tuple_after.switch_edge(m);
            e_tuple_after = e_tuple_after.switch_vertex(m);
            e_tuple_after = e_tuple_after.switch_edge(m);
            e_tuple_after = e_tuple_after.switch_vertex(m);
            e_tuple_after = e_tuple_after.switch_edge(m);
            REQUIRE(tuple_equal(e_tuple, e_tuple_after));
        }
    }

    SECTION("test all tuples generated using faces")
    {
        TriMesh::Tuple f_tuple_after;
        auto faces_tuples = m.generate_tuples_from_faces();
        for (int i = 0; i < faces_tuples.size(); i++) {
            TriMesh::Tuple f_tuple = faces_tuples[i];
            f_tuple_after = f_tuple.switch_vertex(m);
            f_tuple_after = f_tuple_after.switch_edge(m);
            f_tuple_after = f_tuple_after.switch_vertex(m);
            f_tuple_after = f_tuple_after.switch_edge(m);
            f_tuple_after = f_tuple_after.switch_vertex(m);
            f_tuple_after = f_tuple_after.switch_edge(m);
            REQUIRE(tuple_equal(f_tuple, f_tuple_after));
        }
    }
}