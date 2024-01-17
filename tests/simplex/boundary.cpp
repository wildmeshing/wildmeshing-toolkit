#include <catch2/catch_test_macros.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"

#include <wmtk/simplex/boundary.hpp>
using namespace wmtk;
using namespace simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;
TEST_CASE("simplex_boundary", "[simplex_collection][2D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(0, 1, 0));

    SECTION("vertex")
    {
        SimplexCollection bd = boundary(m, Simplex::vertex(t));
        REQUIRE(bd.simplex_vector().size() == 0);
    }
    SECTION("edge")
    {
        SimplexCollection bd = boundary(m, Simplex::edge(t));
        REQUIRE(bd.simplex_vector().size() == 2);
        const std::vector<Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        REQUIRE(v.size() == 2);
        CHECK(m.id(v[0]) == 0);
        CHECK(m.id(v[1]) == 1);

        CHECK(bd.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("face")
    {
        SimplexCollection bd = boundary(m, Simplex::face(t));
        REQUIRE(bd.simplex_vector().size() == 3);
        CHECK(bd.simplex_vector(PrimitiveType::Vertex).size() == 0);


        const std::vector<Simplex> e = bd.simplex_vector(PrimitiveType::Edge);
        CHECK(e.size() == 3);
        SimplexCollection expected_edges(m);
        expected_edges.add(Simplex::edge(m.edge_tuple_between_v1_v2(0, 1, 0)));
        expected_edges.add(Simplex::edge(m.edge_tuple_between_v1_v2(1, 2, 0)));
        expected_edges.add(Simplex::edge(m.edge_tuple_between_v1_v2(2, 0, 0)));
        expected_edges.sort_and_clean();
        const std::vector<Simplex> expected_edge_simplices =
            expected_edges.simplex_vector(PrimitiveType::Edge);
        REQUIRE(e.size() <= 3);
        for (size_t i = 0; i < e.size(); ++i) {
            CHECK(simplex::utils::SimplexComparisons::equal(m, e[i], expected_edge_simplices[i]));
        }

        CHECK(bd.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("tetrahedron")
    {
        SimplexCollection bd = boundary(m, Simplex::tetrahedron(t));
        REQUIRE(bd.simplex_vector().size() == 4);
        CHECK(bd.simplex_vector(PrimitiveType::Vertex).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Edge).size() == 0);


        const std::vector<Simplex> f = bd.simplex_vector(PrimitiveType::Face);
        REQUIRE(f.size() == 4);
        CHECK(m.id(f[0]) == 0);
        CHECK(m.id(f[1]) == 1);
        CHECK(m.id(f[2]) == 2);
        CHECK(m.id(f[3]) == 3);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
}
