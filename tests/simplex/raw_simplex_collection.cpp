
#include <wmtk/simplex/RawSimplex.hpp>
#include <wmtk/simplex/RawSimplexCollection.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <catch2/catch_test_macros.hpp>
#include "tools/TriMesh_examples.hpp"
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/faces_iterable.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
using namespace wmtk;
using namespace simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("raw_simplex_comparison", "[raw_simplex_collection]")
{
    // switching anything in a tuple besides the currently viewed simplex must not change the
    // simplex

    TriMesh m = tests::quad();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PV);
        REQUIRE(vertices.size() == 4);
        for (const Tuple& t : vertices) {
            const RawSimplex s0(m, Simplex::vertex(t));
            const RawSimplex s1(m, Simplex::vertex(m.switch_edge(t)));
            const RawSimplex s_edge(m, Simplex::edge(t));
            CHECK(s0 == s1);
            CHECK_FALSE(s0 == s_edge);
            CHECK(s0 < s_edge);
            if (m.is_boundary_edge(t)) {
                continue;
            }
            const RawSimplex s2(m, Simplex::vertex(m.switch_face(t)));
            CHECK(s0 == s2);
            CHECK(s1 == s2);
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PE);
        REQUIRE(edges.size() == 5);
        for (const Tuple& t : edges) {
            const RawSimplex s0(m, Simplex::edge(t));
            const RawSimplex s1(m, Simplex::edge(m.switch_vertex(t)));
            CHECK_FALSE(s0 < s1);
            CHECK_FALSE(s1 < s0);
            CHECK(s0 == s1);
            if (m.is_boundary_edge(t)) {
                continue;
            }
            const RawSimplex s2(m, Simplex::edge(m.switch_face(t)));
            CHECK_FALSE(s0 < s2);
            CHECK_FALSE(s2 < s0);
            CHECK_FALSE(s1 < s2);
            CHECK_FALSE(s2 < s1);
            CHECK(s0 == s2);
            CHECK(s1 == s2);
        }
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PF);
        REQUIRE(faces.size() == 2);
        for (const Tuple& t : faces) {
            const RawSimplex s0(m, Simplex::face(t));
            const RawSimplex s1(m, Simplex::face(m.switch_vertex(t)));
            CHECK_FALSE(s0 < s1);
            CHECK_FALSE(s1 < s0);
            CHECK(s0 == s1);
            const RawSimplex s2(m, Simplex::face(m.switch_edge(t)));
            CHECK_FALSE(s0 < s2);
            CHECK_FALSE(s2 < s0);
            CHECK_FALSE(s1 < s2);
            CHECK_FALSE(s2 < s1);
            CHECK(s0 == s2);
            CHECK(s1 == s2);
        }
    }
}

TEST_CASE("raw_simplex_collection_sorting", "[raw_simplex_collection]")
{
    TriMesh m = tests::quad();
    const std::vector<Tuple> vertices = m.get_all(PV);
    CHECK(vertices.size() == 4);
    const std::vector<Tuple> edges = m.get_all(PE);
    CHECK(edges.size() == 5);
    const std::vector<Tuple> faces = m.get_all(PF);
    CHECK(faces.size() == 2);

    RawSimplexCollection raw_simplex_collection;
    raw_simplex_collection.add(m, PrimitiveType::Vertex, vertices);
    for (const Tuple& t : edges) {
        raw_simplex_collection.add(m, Simplex::edge(t));
    }
    for (const Tuple& t : faces) {
        raw_simplex_collection.add(m, Simplex::face(t));
    }
    CHECK(raw_simplex_collection.simplex_vector().size() == 11);

    // test sorting and clean-up
    for (const Tuple& t : vertices) {
        raw_simplex_collection.add(m, Simplex::vertex(t));
        break;
    }
    CHECK(raw_simplex_collection.simplex_vector(0).size() == 5);
    CHECK(raw_simplex_collection.simplex_vector(1).size() == 5);
    CHECK(raw_simplex_collection.simplex_vector(2).size() == 2);
    CHECK(raw_simplex_collection.simplex_vector().size() == 12);
    raw_simplex_collection.sort_and_clean();
    CHECK(raw_simplex_collection.simplex_vector().size() == 11);
    CHECK(raw_simplex_collection.simplex_vector(0).size() == 4);
    CHECK(raw_simplex_collection.simplex_vector(1).size() == 5);
    CHECK(raw_simplex_collection.simplex_vector(2).size() == 2);
}

TEST_CASE("raw_simplex_collection_binary_operations", "[raw_simplex_collection]")
{
    RawSimplexCollection sc1, sc2;
    sc1.add(RawSimplex({0}));
    sc1.add(RawSimplex({0, 1}));
    sc1.add(RawSimplex({2, 0}));
    sc1.sort_and_clean();

    CHECK(sc1.contains(RawSimplex({0})));
    CHECK(sc1.contains(RawSimplex({0, 1})));
    CHECK(sc1.contains(RawSimplex({0, 2})));
    CHECK_FALSE(sc1.contains(RawSimplex({1})));

    sc2.add(RawSimplex({1}));
    CHECK(RawSimplexCollection::get_intersection(sc1, sc2).simplex_vector().empty());

    RawSimplexCollection sc_union1 = RawSimplexCollection::get_union(sc1, sc2);
    CHECK(
        sc_union1.simplex_vector().size() ==
        sc1.simplex_vector().size() + sc2.simplex_vector().size());
    CHECK(sc_union1.contains(RawSimplex({0})));
    CHECK(sc_union1.contains(RawSimplex({1})));

    sc2.add(RawSimplex({0}));
    sc2.sort_and_clean();

    RawSimplexCollection sc_union2 = RawSimplexCollection::get_union(sc1, sc2);
    CHECK(RawSimplexCollection::are_simplex_collections_equal(sc_union1, sc_union2));

    RawSimplexCollection sc_inter = RawSimplexCollection::get_intersection(sc1, sc2);
    CHECK(sc_inter.simplex_vector().size() == 1);
    CHECK(sc_inter.contains(RawSimplex({0})));
}

TEST_CASE("raw_simplex_with_invalid_tuple", "[raw_simplex_collection]")
{
    TriMesh m = tests::single_triangle();

    RawSimplexCollection sc;

    for (const Tuple& t : m.get_all(PrimitiveType::Face)) {
        sc.add(m, Simplex::face(t));
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        if (!m.is_boundary_edge(t)) {
            continue;
        }
        std::vector<Tuple> vertices =
            faces_single_dimension_tuples(m, Simplex::edge(t), PrimitiveType::Vertex);
        vertices.emplace_back(Tuple()); // dummy vertex
        RawSimplex simplex(m, vertices);
        sc.add(simplex);
    }
    sc.sort_and_clean();

    CHECK(sc.simplex_vector().size() == 4);
    CHECK(sc.contains(RawSimplex({-1, 0, 1})));
    CHECK(sc.contains(RawSimplex({-1, 0, 2})));
    CHECK(sc.contains(RawSimplex({-1, 1, 2})));
    CHECK(sc.contains(RawSimplex({0, 1, 2})));
}

TEST_CASE("raw_simplex_faces", "[raw_simplex_collection]")
{
    SECTION("without_mesh")
    {
        RawSimplex tet({0, 1, 2, 3});
        CHECK(tet.dimension() == 3);
        RawSimplexCollection tet_faces = tet.faces();
        CHECK(tet_faces.simplex_vector().size() == 14);
        CHECK(tet_faces.contains(RawSimplex({0})));
        CHECK(tet_faces.contains(RawSimplex({1})));
        CHECK(tet_faces.contains(RawSimplex({2})));
        CHECK(tet_faces.contains(RawSimplex({3})));
        CHECK(tet_faces.contains(RawSimplex({0, 1})));
        CHECK(tet_faces.contains(RawSimplex({0, 2})));
        CHECK(tet_faces.contains(RawSimplex({0, 3})));
        CHECK(tet_faces.contains(RawSimplex({1, 2})));
        CHECK(tet_faces.contains(RawSimplex({1, 3})));
        CHECK(tet_faces.contains(RawSimplex({2, 3})));
        CHECK(tet_faces.contains(RawSimplex({0, 1, 2})));
        CHECK(tet_faces.contains(RawSimplex({0, 1, 3})));
        CHECK(tet_faces.contains(RawSimplex({0, 2, 3})));
        CHECK(tet_faces.contains(RawSimplex({1, 2, 3})));

        RawSimplex tri = tet.opposite_face(0);
        CHECK(tri.dimension() == 2);
        CHECK(tri < tet);

        RawSimplexCollection tri_faces = tri.faces();
        CHECK(tri_faces.simplex_vector().size() == 6);
        CHECK(tri_faces.contains(RawSimplex({1})));
        CHECK(tri_faces.contains(RawSimplex({2})));
        CHECK(tri_faces.contains(RawSimplex({3})));
        CHECK(tri_faces.contains(RawSimplex({1, 2})));
        CHECK(tri_faces.contains(RawSimplex({1, 3})));
        CHECK(tri_faces.contains(RawSimplex({2, 3})));

        RawSimplex edge = tri.opposite_face(2);
        CHECK(edge.dimension() == 1);
        RawSimplexCollection edge_faces = edge.faces();
        CHECK(edge_faces.simplex_vector().size() == 2);
        CHECK(edge_faces.contains(RawSimplex({1})));
        CHECK(edge_faces.contains(RawSimplex({3})));

        RawSimplex vertex = edge.opposite_face(3);
        CHECK(vertex.dimension() == 0);
        CHECK(vertex.faces().simplex_vector().empty());

        RawSimplex higher_dim_simplex({0, 1, 2, 3, 4});
        CHECK(higher_dim_simplex.dimension() == 4);
        CHECK_THROWS(higher_dim_simplex.faces());

        RawSimplex opp_edge = tet.opposite_face(edge);
        CHECK(opp_edge.dimension() == 1);
        RawSimplexCollection opp_edge_faces = opp_edge.faces();
        CHECK(opp_edge_faces.simplex_vector().size() == 2);
        CHECK(opp_edge_faces.contains(RawSimplex({0})));
        CHECK(opp_edge_faces.contains(RawSimplex({2})));

        RawSimplex opp_vertex = tet.opposite_face(tri);
        CHECK(opp_vertex.dimension() == 0);
        CHECK(opp_vertex.faces().simplex_vector().empty());
    }
    SECTION("with_mesh")
    {
        TriMesh m = tests::single_triangle();

        RawSimplexCollection sc;
        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            std::vector<Tuple> vertices{t, m.switch_vertex(t), Tuple()};
            RawSimplex s(m, vertices);
            CHECK(s.dimension() == 2);
            RawSimplexCollection s_faces = s.faces();
            CHECK(s_faces.simplex_vector().size() == 6);
            CHECK(s_faces.contains(RawSimplex({-1})));

            RawSimplex opposite_edge = s.opposite_face(m, t);
            CHECK(opposite_edge.dimension() == 1);
            CHECK(opposite_edge.faces().contains(RawSimplex({-1})));

            sc.add(s);
        }

        CHECK(sc.simplex_vector().size() == 3);
        sc.sort_and_clean();
        CHECK(sc.simplex_vector().size() == 3);
    }
}

