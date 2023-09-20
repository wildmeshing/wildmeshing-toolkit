#include <catch2/catch_test_macros.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/coface_cells.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/simplex/simplex_boundary.hpp>
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
            CHECK(m.simplices_are_equal(s0, s1));
            if (m.is_boundary(t)) {
                continue;
            }
            const simplex::Simplex s2(PV, m.switch_tuple(t, PF));
            CHECK(m.simplices_are_equal(s0, s2));
            CHECK(m.simplices_are_equal(s1, s2));
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
        SimplexCollection bd = simplex_boundary(m, simplex::Simplex::vertex(t));
        REQUIRE(bd.simplex_vector().size() == 0);
    }
    SECTION("edge")
    {
        SimplexCollection bd = simplex_boundary(m, simplex::Simplex::edge(t));
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
        SimplexCollection bd = simplex_boundary(m, simplex::Simplex::face(t));
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

TEST_CASE("simplex_coface", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection cc = coface_cells(m, simplex::Simplex::vertex(t));

        REQUIRE(cc.simplex_vector().size() == 6);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 6);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 1);
        CHECK(m.id(cells[2]) == 2);
        CHECK(m.id(cells[3]) == 5);
        CHECK(m.id(cells[4]) == 6);
        CHECK(m.id(cells[5]) == 7);
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 4, 0);

        SimplexCollection cc = coface_cells(m, simplex::Simplex::vertex(t));

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 5);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection cc = coface_cells(m, simplex::Simplex::edge(t));

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 2);
        CHECK(m.id(cells[1]) == 7);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 7, 5);

        SimplexCollection cc = coface_cells(m, simplex::Simplex::edge(t));

        REQUIRE(cc.simplex_vector().size() == 1);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 1);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 5);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection cc = coface_cells(m, simplex::Simplex::face(t));

        REQUIRE(cc.simplex_vector().size() == 1);
        REQUIRE(cc.simplex_vector(PrimitiveType::Face).size() == 1);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 2);
    }
}

TEST_CASE("simplex_open_star", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection os = open_star(m, simplex::Simplex::vertex(t));

        REQUIRE(os.simplex_vector().size() == 13);
        CHECK(os.simplex_vector(PrimitiveType::Face).size() == 6);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 6);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 1);

        const auto& simplices = os.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(t);
        CHECK(m.id(simplices[0]) == m.id(v));

        CHECK(simplex_boundary(m, simplices[1]).contains(v));
        CHECK(simplex_boundary(m, simplices[2]).contains(v));
        CHECK(simplex_boundary(m, simplices[3]).contains(v));
        CHECK(simplex_boundary(m, simplices[4]).contains(v));
        CHECK(simplex_boundary(m, simplices[5]).contains(v));
        CHECK(simplex_boundary(m, simplices[6]).contains(v));

        CHECK(m.id(simplices[7]) == 0);
        CHECK(m.id(simplices[8]) == 1);
        CHECK(m.id(simplices[9]) == 2);
        CHECK(m.id(simplices[10]) == 5);
        CHECK(m.id(simplices[11]) == 6);
        CHECK(m.id(simplices[12]) == 7);
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 4, 0);

        SimplexCollection os = open_star(m, simplex::Simplex::vertex(t));

        REQUIRE(os.simplex_vector().size() == 6);
        CHECK(os.simplex_vector(PrimitiveType::Face).size() == 2);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 3);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 1);

        const auto& simplices = os.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(t);
        CHECK(m.id(simplices[0]) == m.id(v));

        CHECK(simplex_boundary(m, simplices[1]).contains(v));
        CHECK(simplex_boundary(m, simplices[2]).contains(v));
        CHECK(simplex_boundary(m, simplices[3]).contains(v));

        CHECK(m.id(simplices[4]) == 0);
        CHECK(m.id(simplices[5]) == 5);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection os = open_star(m, simplex::Simplex::edge(t));

        REQUIRE(os.simplex_vector().size() == 3);
        CHECK(os.simplex_vector(PrimitiveType::Face).size() == 2);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 0);

        const auto& simplices = os.simplex_vector();

        CHECK(m.id(simplices[0]) == m.id(simplex::Simplex::edge(t)));

        CHECK(m.id(simplices[1]) == 2);
        CHECK(m.id(simplices[2]) == 7);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 7, 5);

        SimplexCollection os = open_star(m, simplex::Simplex::edge(t));

        REQUIRE(os.simplex_vector().size() == 2);
        CHECK(os.simplex_vector(PrimitiveType::Face).size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 0);

        const auto& simplices = os.simplex_vector();

        CHECK(m.id(simplices[0]) == m.id(simplex::Simplex::edge(t)));

        CHECK(m.id(simplices[1]) == 5);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection os = open_star(m, simplex::Simplex::face(t));

        REQUIRE(os.simplex_vector().size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Face).size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 0);

        const auto& simplices = os.simplex_vector();
        CHECK(m.id(simplices[0]) == 2);
    }
}