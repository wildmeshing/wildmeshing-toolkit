#include <catch2/catch_test_macros.hpp>
#include <set>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/top_dimension_cofaces_iterable.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
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


namespace {
    SimplexCollection brute_force_cofaces_single_dimension(const Mesh& m, PrimitiveType pt) {
    }
}

TEST_CASE("simplex_top_dimension_cofaces", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);
        const simplex::Simplex input = simplex::Simplex::vertex(t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 6);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 6);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 1);
        CHECK(m.id(cells[2]) == 2);
        CHECK(m.id(cells[3]) == 5);
        CHECK(m.id(cells[4]) == 6);
        CHECK(m.id(cells[5]) == 7);

        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 4, 0);
        Simplex input = simplex::Simplex::vertex(t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 5);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        Simplex input = simplex::Simplex::edge(t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 2);
        CHECK(m.id(cells[1]) == 7);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 7, 5);

        Simplex input = simplex::Simplex::edge(t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 1);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 1);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 5);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        Simplex input = simplex::Simplex::face(t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 1);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 1);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 2);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }
    }
}

TEST_CASE("simplex_top_dimension_cofaces_iterable", "[simplex_collection][2D]")
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

    TopDimensionCofacesIterable itrb = top_dimension_cofaces_iterable(m, simplex);
    SimplexCollection coll = top_dimension_cofaces(m, simplex);

    SimplexCollection itrb_collection(m);
    for (const Simplex& s : itrb) {
        itrb_collection.add(s);
    }
    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.simplex_vector().size() == coll.simplex_vector().size());

    for (size_t i = 0; i < coll.simplex_vector().size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        check_match_below_simplex_type(m, simplex, coll_s);

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}
TEST_CASE("simplex_cofaces_single_dimension", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);
        const simplex::Simplex input = simplex::Simplex::vertex(t);
        std::vector<Tuple> tc = cofaces_single_dimension_tuples(m, input, PrimitiveType::Edge);
        REQUIRE(tc.size() == 6);

        SimplexCollection sc(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(tc, PrimitiveType::Face));
        sc.sort();
        const auto& cells = sc.simplex_vector();
        std::set<int64_t> target_vids({0, 3, 1, 5, 7, 8});
        std::set<int64_t> vids;
        std::transform(
            cells.begin(),
            cells.end(),
            std::inserter(vids, vids.end()),
            [&](const Simplex& s) {
                return m.id(m.switch_vertex(s.tuple()), PrimitiveType::Vertex);
            });

        CHECK(target_vids == vids);

        // check the lower dimension coface is the same as input
        for (const Tuple& tup : tc) {
            CHECK(m.id(tup, PrimitiveType::Vertex) == m.id(t, PrimitiveType::Vertex));
        }
    }

    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 4, 0);
        const simplex::Simplex input = simplex::Simplex::vertex(t);
        std::vector<Tuple> tc = cofaces_single_dimension_tuples(m, input, PrimitiveType::Edge);
        REQUIRE(tc.size() == 3);
        SimplexCollection sc(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(tc, PrimitiveType::Face));
        sc.sort();

        const auto& cells = sc.simplex_vector();

        // check the lower dimension coface is the same as input
        for (const Tuple& tup : tc) {
            CHECK(m.id(tup, PrimitiveType::Vertex) == m.id(t, PrimitiveType::Vertex));
        }

        CHECK(m.id(m.switch_vertex(cells[0].tuple()), PrimitiveType::Vertex) == 0);
        CHECK(m.id(m.switch_vertex(cells[1].tuple()), PrimitiveType::Vertex) == 4);
        CHECK(m.id(m.switch_vertex(cells[2].tuple()), PrimitiveType::Vertex) == 7);
    }
}

