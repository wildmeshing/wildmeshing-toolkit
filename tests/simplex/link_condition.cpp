
#include <catch2/catch_test_macros.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/simplex/link_condition.hpp>
#include <wmtk/simplex/link_iterable.hpp>
#include "test_utils.hpp"
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("simplex_link_2d", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection sc = link(m, Simplex::vertex(t));

        REQUIRE(sc.simplex_vector().size() == 12);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 6);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 6);

        const auto& simplices = sc.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(t);
        CHECK(m.id(simplices[0]) == 0);
        CHECK(m.id(simplices[1]) == 1);
        CHECK(m.id(simplices[2]) == 3);
        CHECK(m.id(simplices[3]) == 5);
        CHECK(m.id(simplices[4]) == 7);
        CHECK(m.id(simplices[5]) == 8);

        for (size_t i = 6; i < 12; ++i) {
            const Simplex& e = simplices[i];
            const Tuple center = m.switch_vertex(m.next_edge(e.tuple()));
            CHECK(simplex::utils::SimplexComparisons::equal(m, v, Simplex::vertex(center)));
        }
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 4, 0);

        SimplexCollection sc = link(m, Simplex::vertex(t));

        REQUIRE(sc.simplex_vector().size() == 5);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 2);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 3);

        const auto& simplices = sc.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(t);
        CHECK(m.id(simplices[0]) == 0);
        CHECK(m.id(simplices[1]) == 4);
        CHECK(m.id(simplices[2]) == 7);

        for (size_t i = 3; i < 5; ++i) {
            const Simplex& e = simplices[i];
            const Tuple center = m.switch_vertex(m.next_edge(e.tuple()));
            CHECK(simplex::utils::SimplexComparisons::equal(m, v, Simplex::vertex(center)));
        }
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection sc = link(m, Simplex::edge(t));

        REQUIRE(sc.simplex_vector().size() == 2);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 2);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 1);
        CHECK(m.id(simplices[1]) == 8);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 7, 5);

        SimplexCollection sc = link(m, Simplex::edge(t));

        REQUIRE(sc.simplex_vector().size() == 1);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 1);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 4);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection cs = link(m, Simplex::face(t));

        REQUIRE(cs.simplex_vector().size() == 0);
    }
}

TEST_CASE("simplex_link_iterable", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    Simplex simplex = Simplex::vertex({});

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);
        simplex = Simplex::vertex(t);
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 4, 0);
        simplex = Simplex::vertex(t);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);
        simplex = Simplex::edge(t);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 7, 5);
        simplex = Simplex::edge(t);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);
        simplex = Simplex::face(t);
    }

    LinkIterable itrb = link_iterable(m, simplex);
    SimplexCollection coll = link(m, simplex);

    SimplexCollection itrb_collection(m);
    for (const Simplex& s : itrb) {
        itrb_collection.add(s);
    }
    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.simplex_vector().size() == coll.simplex_vector().size());

    for (size_t i = 0; i < coll.simplex_vector().size(); ++i) {
        CHECK(simplex::utils::SimplexComparisons::equal(
            m,
            itrb_collection.simplex_vector()[i],
            coll.simplex_vector()[i]));
    }
}


TEST_CASE("simplex_link_condtion_edgemesh", "[simplex_collection]")
{
    SECTION("cases should succeed")
    {
        tests::DEBUG_EdgeMesh m0 = tests::two_segments();
        tests::DEBUG_EdgeMesh m1 = tests::loop_lines();
        tests::DEBUG_EdgeMesh m2 = tests::two_line_loop();

        int64_t hash = 0;
        Tuple t(0, -1, -1, 0, hash);
        REQUIRE(link_condition(m0, t) == true);
        REQUIRE(link_condition(m1, t) == true);
        REQUIRE(link_condition(m2, t) == true);
    }

    SECTION("cases should fail")
    {
        tests::DEBUG_EdgeMesh m0 = tests::single_line();
        tests::DEBUG_EdgeMesh m1 = tests::self_loop();

        int64_t hash = 0;
        Tuple t(0, -1, -1, 0, hash);
        REQUIRE(link_condition(m0, t) == false);
        REQUIRE(link_condition(m1, t) == false);
    }
}

TEST_CASE("simplex_link_condtion_trimesh", "[simplex_collection]")
{
    SECTION("case three neighbors")
    {
        tests::DEBUG_TriMesh m;
        m = tests::three_neighbors();
        // get the tuple point to V(0), E(01), F(012)
        int64_t hash = 0;
        Tuple t(0, 2, -1, 1, hash);
        REQUIRE(link_condition(m, t) == false);
    }

    SECTION("case two neighbors")
    {
        tests::DEBUG_TriMesh m;
        m = tests::two_neighbors();
        Tuple t1 = m.edge_tuple_between_v1_v2(0, 1, 0);
        Tuple t2 = m.edge_tuple_between_v1_v2(1, 2, 0);
        Tuple t3 = m.edge_tuple_between_v1_v2(2, 0, 2);
        REQUIRE(link_condition(m, t1) == false);
        REQUIRE(link_condition(m, t2) == true);
        REQUIRE(link_condition(m, t3) == false);
    }

    SECTION("case nine_triangles_with_a_hole")
    {
        tests::DEBUG_TriMesh m;
        m = tests::nine_triangles_with_a_hole();
        Tuple t1 = m.edge_tuple_between_v1_v2(1, 2, 0);
        Tuple t2 = m.edge_tuple_between_v1_v2(2, 4, 2);
        Tuple t3 = m.edge_tuple_between_v1_v2(1, 6, 3);
        REQUIRE(link_condition(m, t1) == false);
        REQUIRE(link_condition(m, t2) == false);
        REQUIRE(link_condition(m, t3) == true);
    }
}

TEST_CASE("are_simplex_collections_equal", "[simplex_collection]")
{
    // TODO: test should be extended to cases where the two sets are not equal
    TriMesh m = tests::quad();

    const std::vector<Tuple> vertices = m.get_all(PV);
    const std::vector<Tuple> edges = m.get_all(PE);
    const std::vector<Tuple> faces = m.get_all(PF);

    const int key = 2;

    SimplexCollection sc1(m);
    SimplexCollection sc2(m);

    for (int64_t i = 0; i < 6; i++) {
        if (i % key == 1) continue;
        if (i < vertices.size()) {
            sc1.add(Simplex(PV, vertices[i]));
        }
        if (i < edges.size()) {
            sc1.add(Simplex(PE, edges[i]));
        }
        if (i < faces.size()) {
            sc1.add(Simplex(PF, faces[i]));
        }
    }

    for (int64_t i = 5; i >= 0; i--) {
        if (i % key == 1) continue;
        if (i < vertices.size()) {
            sc2.add(Simplex(PV, vertices[i]));
        }
        if (i < edges.size()) {
            sc2.add(Simplex(PE, edges[i]));
        }
        if (i < faces.size()) {
            sc2.add(Simplex(PF, faces[i]));
        }
    }
    sc1.sort_and_clean();
    sc2.sort_and_clean();
    REQUIRE(SimplexCollection::are_simplex_collections_equal(sc1, sc2) == true);
}

TEST_CASE("simplex_link_condtion_tetmesh", "[simplex_collection]")
{
    SECTION("one tet")
    {
        tests_3d::DEBUG_TetMesh m;
        m = tests_3d::single_tet();
        std::array<Tuple, 6> e;
        // cannot collapse
        e[0] = m.edge_tuple_from_vids(0, 1);
        e[1] = m.edge_tuple_from_vids(0, 2);
        e[2] = m.edge_tuple_from_vids(0, 3);
        e[3] = m.edge_tuple_from_vids(1, 2);
        e[4] = m.edge_tuple_from_vids(1, 3);
        e[5] = m.edge_tuple_from_vids(2, 3);

        for (size_t i = 0; i < 6; ++i) {
            CHECK_FALSE(link_condition(m, e[i]));
        }
    }

    SECTION("one ear")
    {
        tests_3d::DEBUG_TetMesh m;
        m = tests_3d::one_ear();
        std::array<Tuple, 9> e;
        // cannot collapse
        e[0] = m.edge_tuple_from_vids(2, 3);
        e[1] = m.edge_tuple_from_vids(0, 2);
        e[2] = m.edge_tuple_from_vids(0, 3);
        // can collapse
        e[3] = m.edge_tuple_from_vids(1, 0);
        e[4] = m.edge_tuple_from_vids(1, 2);
        e[5] = m.edge_tuple_from_vids(1, 3);
        e[6] = m.edge_tuple_from_vids(4, 0);
        e[7] = m.edge_tuple_from_vids(4, 2);
        e[8] = m.edge_tuple_from_vids(4, 3);

        for (size_t i = 0; i < 3; ++i) {
            CHECK_FALSE(link_condition(m, e[i]));
        }
        for (size_t i = 3; i < 9; ++i) {
            CHECK(link_condition(m, e[i]));
        }
    }

    SECTION("two ears")
    {
        tests_3d::DEBUG_TetMesh m;
        m = tests_3d::two_ears();
        std::array<Tuple, 12> e;
        // cannot collapse
        e[7] = m.edge_tuple_from_vids(0, 2);
        e[8] = m.edge_tuple_from_vids(0, 3);
        e[9] = m.edge_tuple_from_vids(0, 1);
        e[10] = m.edge_tuple_from_vids(1, 3);
        e[11] = m.edge_tuple_from_vids(2, 3);
        // can collapse
        e[0] = m.edge_tuple_from_vids(1, 2);
        e[1] = m.edge_tuple_from_vids(5, 0);
        e[2] = m.edge_tuple_from_vids(5, 1);
        e[3] = m.edge_tuple_from_vids(5, 3);
        e[4] = m.edge_tuple_from_vids(4, 2);
        e[5] = m.edge_tuple_from_vids(4, 3);
        e[6] = m.edge_tuple_from_vids(4, 0);

        for (size_t i = 7; i < 12; ++i) {
            CHECK_FALSE(link_condition(m, e[i]));
        }
        for (size_t i = 0; i < 7; ++i) {
            CHECK(link_condition(m, e[i]));
        }
    }

    SECTION("three incident tets")
    {
        tests_3d::DEBUG_TetMesh m;
        m = tests_3d::three_incident_tets();
        std::array<Tuple, 12> e;
        // cannot collapse
        e[7] = m.edge_tuple_from_vids(2, 3);
        e[8] = m.edge_tuple_from_vids(0, 3);
        e[9] = m.edge_tuple_from_vids(0, 2);
        e[10] = m.edge_tuple_from_vids(4, 3);
        e[11] = m.edge_tuple_from_vids(4, 2);
        // can collapse
        e[0] = m.edge_tuple_from_vids(1, 2);
        e[1] = m.edge_tuple_from_vids(1, 3);
        e[2] = m.edge_tuple_from_vids(1, 0);
        e[3] = m.edge_tuple_from_vids(5, 3);
        e[4] = m.edge_tuple_from_vids(5, 2);
        e[5] = m.edge_tuple_from_vids(5, 4);
        e[6] = m.edge_tuple_from_vids(4, 0);

        for (size_t i = 7; i < 12; ++i) {
            CHECK_FALSE(link_condition(m, e[i]));
        }
        for (size_t i = 0; i < 7; ++i) {
            CHECK(link_condition(m, e[i]));
        }
    }

    SECTION("six cycle tets")
    {
        tests_3d::DEBUG_TetMesh m;
        m = tests_3d::six_cycle_tets();
        std::array<Tuple, 19> e;
        // cannot collapse
        e[0] = m.edge_tuple_from_vids(2, 3);

        // can collapse
        e[1] = m.edge_tuple_from_vids(4, 0);
        e[2] = m.edge_tuple_from_vids(5, 4);
        e[3] = m.edge_tuple_from_vids(7, 5);
        e[4] = m.edge_tuple_from_vids(6, 7);
        e[5] = m.edge_tuple_from_vids(1, 6);
        e[6] = m.edge_tuple_from_vids(0, 1);

        e[7] = m.edge_tuple_from_vids(0, 2);
        e[8] = m.edge_tuple_from_vids(0, 3);
        e[9] = m.edge_tuple_from_vids(1, 2);
        e[10] = m.edge_tuple_from_vids(1, 3);
        e[11] = m.edge_tuple_from_vids(4, 2);
        e[12] = m.edge_tuple_from_vids(4, 3);
        e[13] = m.edge_tuple_from_vids(5, 2);
        e[14] = m.edge_tuple_from_vids(5, 3);
        e[15] = m.edge_tuple_from_vids(6, 2);
        e[16] = m.edge_tuple_from_vids(6, 3);
        e[17] = m.edge_tuple_from_vids(7, 2);
        e[18] = m.edge_tuple_from_vids(7, 3);


        for (int i = 0; i < 1; ++i) {
            CHECK_FALSE(link_condition(m, e[i]));
        }
        for (int i = 1; i < 19; ++i) {
            CHECK(link_condition(m, e[i]));
        }
    }
}

TEST_CASE("simplex_link_3d", "[simplex_collection][3D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::two_by_two_by_two_grids_tets();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(13, 14, 17);

        SimplexCollection sc = link(m, Simplex::vertex(t));

        REQUIRE(sc.simplex_vector().size() == 98);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 32);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 48);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 18);

        const auto& simplices = sc.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(t);
        CHECK(m.id(simplices[0]) == 1);
        CHECK(m.id(simplices[1]) == 3);
        CHECK(m.id(simplices[2]) == 4);
        CHECK(m.id(simplices[3]) == 5);
        CHECK(m.id(simplices[4]) == 7);
        CHECK(m.id(simplices[5]) == 9);
        CHECK(m.id(simplices[6]) == 10);
        CHECK(m.id(simplices[7]) == 11);
        CHECK(m.id(simplices[8]) == 12);
        CHECK(m.id(simplices[9]) == 14);
        CHECK(m.id(simplices[10]) == 15);
        CHECK(m.id(simplices[11]) == 16);
        CHECK(m.id(simplices[12]) == 17);
        CHECK(m.id(simplices[13]) == 19);
        CHECK(m.id(simplices[14]) == 21);
        CHECK(m.id(simplices[15]) == 22);
        CHECK(m.id(simplices[16]) == 23);
        CHECK(m.id(simplices[17]) == 25);

        for (size_t i = 18; i < 48; ++i) {
            const Simplex& e = simplices[i];
            int id0 = m.id(e.tuple(), PrimitiveType::Vertex);
            int id1 = m.id(m.switch_vertex(e.tuple()), PrimitiveType::Vertex);
            m.face_tuple_from_vids(id0, id1, 13);
        }

        /**************HERE IS A PROBLEM, MAYBE A BUG****************/
        int sum = 0;
        for (size_t i = 48; i < 98; ++i) {
            const Simplex& f = simplices[i];
            int id0 = m.id(f.tuple(), PrimitiveType::Vertex);
            int id1 = m.id(m.switch_vertex(f.tuple()), PrimitiveType::Vertex);
            int id2 = m.id(m.switch_vertex(m.switch_edge(f.tuple())), PrimitiveType::Vertex);
            if (id0 == 13 || id1 == 13 || id2 == 13) {
                sum++;
            }
            // m.tet_tuple_from_vids();
        }
        sum;
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(14, 13, 8);

        CHECK(m.id(t, PrimitiveType::Vertex) == 14);

        SimplexCollection sc = link(m, Simplex::vertex(t));

        REQUIRE(sc.simplex_vector().size() == 17);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 4);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 8);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 5);

        const auto& simplices = sc.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(t);
        CHECK(m.id(simplices[0]) == 5);
        CHECK(m.id(simplices[1]) == 11);
        CHECK(m.id(simplices[2]) == 13);
        CHECK(m.id(simplices[3]) == 17);
        CHECK(m.id(simplices[4]) == 23);

        for (size_t i = 5; i < 13; ++i) {
            const Simplex& e = simplices[i];
            int id0 = m.id(e.tuple(), PrimitiveType::Vertex);
            int id1 = m.id(m.switch_vertex(e.tuple()), PrimitiveType::Vertex);
            m.face_tuple_from_vids(id0, id1, 14);
        }

        for (size_t i = 13; i < 17; ++i) {
            const Simplex& f = simplices[i];
            int id0 = m.id(f.tuple(), PrimitiveType::Vertex);
            int id1 = m.id(m.switch_vertex(f.tuple()), PrimitiveType::Vertex);
            int id2 = m.id(m.switch_vertex(m.switch_edge(f.tuple())), PrimitiveType::Vertex);
            m.tet_tuple_from_vids(id0, id1, id2, 14);
        }
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(9, 13, 2);

        SimplexCollection sc = link(m, Simplex::edge(t));

        REQUIRE(sc.simplex_vector().size() == 12);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 6);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 6);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 1);
        CHECK(m.id(simplices[1]) == 3);
        CHECK(m.id(simplices[2]) == 10);
        CHECK(m.id(simplices[3]) == 12);
        CHECK(m.id(simplices[4]) == 19);
        CHECK(m.id(simplices[5]) == 21);

        for (size_t i = 6; i < 12; ++i) {
            const Simplex& e = simplices[i];
            int id0 = m.id(e.tuple(), PrimitiveType::Vertex);
            int id1 = m.id(m.switch_vertex(e.tuple()), PrimitiveType::Vertex);
            m.tet_tuple_from_vids(id0, id1, 9, 13);
        }
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(1, 3, 0);

        SimplexCollection sc = link(m, Simplex::edge(t));

        REQUIRE(sc.simplex_vector().size() == 7);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 3);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 4);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 0);
        CHECK(m.id(simplices[1]) == 4);
        CHECK(m.id(simplices[2]) == 9);
        CHECK(m.id(simplices[3]) == 13);

        for (size_t i = 4; i < 7; ++i) {
            const Simplex& e = simplices[i];
            int id0 = m.id(e.tuple(), PrimitiveType::Vertex);
            int id1 = m.id(m.switch_vertex(e.tuple()), PrimitiveType::Vertex);
            m.tet_tuple_from_vids(id0, id1, 1, 3);
        }
    }
    SECTION("face_interior")
    {
        const Tuple t = m.face_tuple_from_vids(9, 10, 13);

        SimplexCollection sc = link(m, Simplex::face(t));

        REQUIRE(sc.simplex_vector().size() == 2);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 2);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 1);
        CHECK(m.id(simplices[1]) == 19);
    }
    SECTION("face_boundary")
    {
        const Tuple t = m.face_tuple_from_vids(0, 1, 3);

        SimplexCollection sc = link(m, Simplex::face(t));

        REQUIRE(sc.simplex_vector().size() == 1);
        CHECK(sc.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 1);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 9);
    }
}

TEST_CASE("simplex_link_iterable_3d", "[simplex_collection][3D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::two_by_two_by_two_grids_tets();

    Simplex simplex = Simplex::vertex({});

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(13, 12, 2);
        simplex = Simplex::vertex(t);
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(1, 4, 1);
        simplex = Simplex::vertex(t);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(13, 12, 2);
        simplex = Simplex::edge(t);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);
        simplex = Simplex::edge(t);
    }
    SECTION("face_interior")
    {
        const Tuple t = m.face_tuple_from_vids(19, 21, 13);
        simplex = Simplex::face(t);
    }
    SECTION("face_boundary")
    {
        const Tuple t = m.face_tuple_from_vids(0, 1, 3);
        simplex = Simplex::face(t);
    }

    LinkIterable itrb = link_iterable(m, simplex);
    SimplexCollection coll = link(m, simplex);

    SimplexCollection itrb_collection(m);
    for (const Simplex& s : itrb) {
        itrb_collection.add(s);
    }
    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.simplex_vector().size() == coll.simplex_vector().size());

    for (size_t i = 0; i < coll.simplex_vector().size(); ++i) {
        CHECK(simplex::utils::SimplexComparisons::equal(
            m,
            itrb_collection.simplex_vector()[i],
            coll.simplex_vector()[i]));
    }
}
