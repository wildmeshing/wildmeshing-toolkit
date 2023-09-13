#include <catch2/catch_test_macros.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexBoundary.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("simplex_comparison", "[simplex_collection][2D]")
{
    // switching anything in a tuple besides the currently viewed simplex must not change the
    // simplex

    TriMesh m = tests::quad();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PV);
        REQUIRE(vertices.size() == 4);
        for (const Tuple& t : vertices) {
            const simplex::Simplex s0(PV, t);
            const simplex::Simplex s1(PV, m.switch_tuple(t, PE));
            CHECK(m.simplex_is_equal(s0, s1));
            if (m.is_boundary(t)) {
                continue;
            }
            const simplex::Simplex s2(PV, m.switch_tuple(t, PF));
            CHECK(m.simplex_is_equal(s0, s2));
            CHECK(m.simplex_is_equal(s1, s2));
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PE);
        REQUIRE(edges.size() == 5);
        for (const Tuple& t : edges) {
            const simplex::Simplex s0(PE, t);
            const simplex::Simplex s1(PE, m.switch_tuple(t, PV));
            CHECK_FALSE(m.simplex_is_less(s0, s1));
            CHECK_FALSE(m.simplex_is_less(s1, s0));
            if (m.is_boundary(t)) {
                continue;
            }
            const simplex::Simplex s2(PE, m.switch_tuple(t, PF));
            CHECK_FALSE(m.simplex_is_less(s0, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s0));
            CHECK_FALSE(m.simplex_is_less(s1, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s1));
        }
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PF);
        REQUIRE(faces.size() == 2);
        for (const Tuple& t : faces) {
            const simplex::Simplex s0(PF, t);
            const simplex::Simplex s1(PF, m.switch_tuple(t, PV));
            CHECK_FALSE(m.simplex_is_less(s0, s1));
            CHECK_FALSE(m.simplex_is_less(s1, s0));
            const simplex::Simplex s2(PF, m.switch_tuple(t, PE));
            CHECK_FALSE(m.simplex_is_less(s0, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s0));
            CHECK_FALSE(m.simplex_is_less(s1, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s1));
        }
    }
}

TEST_CASE("simplex_collection_sorting", "[simplex_collection][2D]")
{
    TriMesh m = tests::quad();
    const std::vector<Tuple> vertices = m.get_all(PV);
    REQUIRE(vertices.size() == 4);
    const std::vector<Tuple> edges = m.get_all(PE);
    REQUIRE(edges.size() == 5);
    const std::vector<Tuple> faces = m.get_all(PF);
    REQUIRE(faces.size() == 2);

    SimplexCollection simplex_collection(m);
    for (const auto& t : vertices) {
        simplex_collection.add(simplex::Simplex::vertex(t));
    }
    for (const auto& t : edges) {
        simplex_collection.add(simplex::Simplex::edge(t));
    }
    for (const auto& t : faces) {
        simplex_collection.add(simplex::Simplex::face(t));
    }
    REQUIRE(simplex_collection.simplex_vector().size() == 11);

    // test sorting and clean-up
    for (const auto& t : vertices) {
        simplex_collection.add(simplex::Simplex::vertex(t));
        break;
    }
    REQUIRE(simplex_collection.simplex_vector().size() == 12);
    simplex_collection.sort_and_clean();
    REQUIRE(simplex_collection.simplex_vector().size() == 11);
}

TEST_CASE("simplex_boundary", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::single_triangle();

    const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

    SECTION("vertex")
    {
        SimplexBoundary bd(m, simplex::Simplex::vertex(t));
        REQUIRE(bd.simplex_vector().size() == 0);
    }
    SECTION("edge")
    {
        SimplexBoundary bd(m, simplex::Simplex::edge(t));
        REQUIRE(bd.simplex_vector().size() == 2);
        const std::vector<simplex::Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        REQUIRE(v.size() == 2);
        CHECK(m.id(v[0]) == 0);
        CHECK(m.id(v[1]) == 1);

        CHECK(bd.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("face")
    {
        SimplexBoundary bd(m, simplex::Simplex::face(t));
        REQUIRE(bd.simplex_vector().size() == 6);
        const std::vector<simplex::Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        CHECK(v.size() == 3);
        CHECK(m.id(v[0]) == 0);
        CHECK(m.id(v[1]) == 1);
        CHECK(m.id(v[2]) == 2);

        const std::vector<simplex::Simplex> e = bd.simplex_vector(PrimitiveType::Edge);
        CHECK(e.size() == 3);
        CHECK(m.id(e[0]) == 0);
        CHECK(m.id(e[1]) == 1);
        CHECK(m.id(e[2]) == 2);

        CHECK(bd.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
}