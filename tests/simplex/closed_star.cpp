

#include <catch2/catch_test_macros.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/closed_star_iterable.hpp>
#include <wmtk/simplex/faces.hpp>
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
TEST_CASE("simplex_closed_star", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection cs = closed_star(m, Simplex::vertex(t));

        REQUIRE(cs.simplex_vector().size() == 25);
        CHECK(cs.simplex_vector(PrimitiveType::Face).size() == 6);
        CHECK(cs.simplex_vector(PrimitiveType::Edge).size() == 12);
        CHECK(cs.simplex_vector(PrimitiveType::Vertex).size() == 7);

        const auto& simplices = cs.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(t);
        CHECK(m.id(simplices[0]) == 0);
        CHECK(m.id(simplices[1]) == 1);
        CHECK(m.id(simplices[2]) == 3);
        CHECK(m.id(simplices[3]) == 4);
        CHECK(m.id(simplices[4]) == 5);
        CHECK(m.id(simplices[5]) == 7);
        CHECK(m.id(simplices[6]) == 8);

        for (size_t i = 7; i < 19; ++i) {
            const Simplex& e = simplices[i];
            const Tuple center = m.switch_vertex(m.next_edge(e.tuple()));
            CHECK(
                (faces(m, e).contains(v) ||
                 simplex::utils::SimplexComparisons::equal(m, v, Simplex::vertex(center))));
        }

        CHECK(m.id(simplices[19]) == 0);
        CHECK(m.id(simplices[20]) == 1);
        CHECK(m.id(simplices[21]) == 2);
        CHECK(m.id(simplices[22]) == 5);
        CHECK(m.id(simplices[23]) == 6);
        CHECK(m.id(simplices[24]) == 7);
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 4, 0);

        SimplexCollection cs = closed_star(m, Simplex::vertex(t));

        REQUIRE(cs.simplex_vector().size() == 11);
        CHECK(cs.simplex_vector(PrimitiveType::Face).size() == 2);
        CHECK(cs.simplex_vector(PrimitiveType::Edge).size() == 5);
        CHECK(cs.simplex_vector(PrimitiveType::Vertex).size() == 4);

        const auto& simplices = cs.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(t);
        CHECK(m.id(simplices[0]) == 0);
        CHECK(m.id(simplices[1]) == 3);
        CHECK(m.id(simplices[2]) == 4);
        CHECK(m.id(simplices[3]) == 7);

        for (size_t i = 4; i < 9; ++i) {
            const Simplex& e = simplices[i];
            const Tuple center = m.switch_vertex(m.next_edge(e.tuple()));
            CHECK(
                (faces(m, e).contains(v) ||
                 simplex::utils::SimplexComparisons::equal(m, v, Simplex::vertex(center))));
        }

        CHECK(m.id(simplices[9]) == 0);
        CHECK(m.id(simplices[10]) == 5);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection cs = closed_star(m, Simplex::edge(t));

        REQUIRE(cs.simplex_vector().size() == 11);
        CHECK(cs.simplex_vector(PrimitiveType::Face).size() == 2);
        CHECK(cs.simplex_vector(PrimitiveType::Edge).size() == 5);
        CHECK(cs.simplex_vector(PrimitiveType::Vertex).size() == 4);

        const auto& simplices = cs.simplex_vector();

        CHECK(m.id(simplices[0]) == 1);
        CHECK(m.id(simplices[1]) == 4);
        CHECK(m.id(simplices[2]) == 5);
        CHECK(m.id(simplices[3]) == 8);

        SimplexCollection t_bd = faces(m, Simplex::edge(t));

        for (size_t i = 4; i < 9; ++i) {
            const Simplex& e = simplices[i];
            SimplexCollection e_bd = faces(m, e);
            SimplexCollection bd_intersection = SimplexCollection::get_intersection(e_bd, t_bd);
            CHECK(
                (simplex::utils::SimplexComparisons::equal(m, Simplex::edge(t), e) ||
                 bd_intersection.simplex_vector().size() == 1));
        }

        CHECK(m.id(simplices[9]) == 2);
        CHECK(m.id(simplices[10]) == 7);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 7, 5);

        SimplexCollection cs = closed_star(m, Simplex::edge(t));

        REQUIRE(cs.simplex_vector().size() == 7);
        CHECK(cs.simplex_vector(PrimitiveType::Face).size() == 1);
        CHECK(cs.simplex_vector(PrimitiveType::Edge).size() == 3);
        CHECK(cs.simplex_vector(PrimitiveType::Vertex).size() == 3);

        const auto& simplices = cs.simplex_vector();

        CHECK(m.id(simplices[0]) == 3);
        CHECK(m.id(simplices[1]) == 4);
        CHECK(m.id(simplices[2]) == 7);

        SimplexCollection t_bd = faces(m, Simplex::edge(t));

        for (size_t i = 3; i < 6; ++i) {
            const Simplex& e = simplices[i];
            SimplexCollection e_bd = faces(m, e);
            SimplexCollection bd_intersection = SimplexCollection::get_intersection(e_bd, t_bd);
            CHECK(
                (simplex::utils::SimplexComparisons::equal(m, Simplex::edge(t), e) ||
                 bd_intersection.simplex_vector().size() == 1));
        }

        CHECK(m.id(simplices[6]) == 5);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection cs = closed_star(m, Simplex::face(t));

        REQUIRE(cs.simplex_vector().size() == 7);
        CHECK(cs.simplex_vector(PrimitiveType::Face).size() == 1);
        CHECK(cs.simplex_vector(PrimitiveType::Edge).size() == 3);
        CHECK(cs.simplex_vector(PrimitiveType::Vertex).size() == 3);

        const auto& simplices = cs.simplex_vector();

        CHECK(m.id(simplices[0]) == 1);
        CHECK(m.id(simplices[1]) == 4);
        CHECK(m.id(simplices[2]) == 5);

        for (size_t i = 3; i < 6; ++i) {
            const Simplex& e = simplices[i];
            CHECK(simplex::utils::SimplexComparisons::equal(
                m,
                Simplex::face(t),
                Simplex::face(e.tuple())));
        }

        CHECK(m.id(simplices[6]) == 2);
    }
}

TEST_CASE("simplex_closed_star_iterable", "[simplex_collection][2D]")
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

    ClosedStarIterable itrb = closed_star_iterable(m, simplex);
    SimplexCollection coll = closed_star(m, simplex);

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
