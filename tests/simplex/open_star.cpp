
#include <catch2/catch_test_macros.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/simplex/open_star_iterable.hpp>
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

        CHECK(faces(m, simplices[1]).contains(v));
        CHECK(faces(m, simplices[2]).contains(v));
        CHECK(faces(m, simplices[3]).contains(v));
        CHECK(faces(m, simplices[4]).contains(v));
        CHECK(faces(m, simplices[5]).contains(v));
        CHECK(faces(m, simplices[6]).contains(v));

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

        CHECK(faces(m, simplices[1]).contains(v));
        CHECK(faces(m, simplices[2]).contains(v));
        CHECK(faces(m, simplices[3]).contains(v));

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
TEST_CASE("simplex_open_star_trimesh", "[simplex_collection][2D]")
{
    std::unique_ptr<tests::DEBUG_TriMesh> mp =
        std::make_unique<tests::DEBUG_TriMesh>(tests::hex_plus_two());
    tests::DEBUG_TriMesh& m = *mp;
    Mesh& mm = *dynamic_cast<Mesh*>(mp.get());

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::vertex(t));
        SimplexCollection os_m = open_star(mm, simplex::Simplex::vertex(t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 4, 0);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::vertex(t));
        SimplexCollection os_m = open_star(mm, simplex::Simplex::vertex(t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::edge(t));
        SimplexCollection os_m = open_star(mm, simplex::Simplex::edge(t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(3, 7, 5);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::edge(t));
        SimplexCollection os_m = open_star(mm, simplex::Simplex::edge(t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_between_v1_v2(4, 5, 2);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::face(t));
        SimplexCollection os_m = open_star(mm, simplex::Simplex::face(t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
}

TEST_CASE("simplex_open_star_iterable", "[simplex_collection][2D]")
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

    OpenStarIterable itrb = open_star_iterable(m, simplex);
    SimplexCollection coll = open_star(m, simplex);

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
