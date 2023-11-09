#include <catch2/catch_test_macros.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/boundary.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/closed_star_iterable.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/faces_iterable.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/simplex/link_iterable.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/simplex/open_star_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/top_dimension_cofaces_iterable.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"

#include <array>

using namespace wmtk;
using namespace simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
// constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;
namespace {
template <typename MeshType> // use a DEBUG mesh type
void check_match_below_simplex_type(const MeshType& mesh, const Simplex& a, const Simplex& b)
{
    PrimitiveType min_type = std::min(a.primitive_type(), b.primitive_type());

    for (int i = 0; i <= get_primitive_type_id(min_type); ++i) {
        PrimitiveType cur_primitive_type = static_cast<PrimitiveType>(i);
        CHECK(mesh.id(a.tuple(), cur_primitive_type) == mesh.id(b.tuple(), cur_primitive_type));
    }
}
} // namespace

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

TEST_CASE("simplex_faces", "[simplex_collection][2D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(0, 1, 0));

    SECTION("vertex")
    {
        SimplexCollection bd = faces(m, Simplex::vertex(t));
        REQUIRE(bd.simplex_vector().size() == 0);
    }
    SECTION("edge")
    {
        SimplexCollection bd = faces(m, Simplex::edge(t));
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
        SimplexCollection bd = faces(m, Simplex::face(t));
        REQUIRE(bd.simplex_vector().size() == 6);
        const std::vector<Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        CHECK(v.size() == 3);
        for (size_t i = 0; i < v.size(); ++i) {
            CHECK(m.id(v[i]) == i);
        }

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
            CHECK(m.simplices_are_equal(e[i], expected_edge_simplices[i]));
        }

        CHECK(bd.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("tetrahedron")
    {
        SimplexCollection bd = faces(m, Simplex::tetrahedron(t));
        REQUIRE(bd.simplex_vector().size() == 14);
        const std::vector<Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        CHECK(v.size() == 4);
        for (size_t i = 0; i < v.size(); ++i) {
            CHECK(m.id(v[i]) == i);
        }

        const std::vector<Simplex> e = bd.simplex_vector(PrimitiveType::Edge);
        CHECK(e.size() == 6);
        for (size_t i = 0; i < e.size(); ++i) {
            CHECK(m.id(e[i]) == i);
        }


        const std::vector<Simplex> f = bd.simplex_vector(PrimitiveType::Face);
        CHECK(f.size() == 4);
        for (size_t i = 0; i < f.size(); ++i) {
            CHECK(m.id(f[i]) == i);
        }
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
}

TEST_CASE("simplex_faces_iterable", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::single_triangle();

    Simplex simplex = Simplex::vertex({});

    const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

    SECTION("vertex")
    {
        simplex = Simplex::vertex(t);
    }
    SECTION("edge")
    {
        simplex = Simplex::edge(t);
    }
    SECTION("face")
    {
        simplex = Simplex::face(t);
    }

    FacesIterable itrb = faces_iterable(m, simplex);
    SimplexCollection coll = faces(m, simplex);

    SimplexCollection itrb_collection(m);
    for (const Simplex& s : itrb) {
        itrb_collection.add(s);
    }
    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.simplex_vector().size() == coll.simplex_vector().size());

    for (size_t i = 0; i < coll.simplex_vector().size(); ++i) {
        CHECK(m.simplices_are_equal(itrb_collection.simplex_vector()[i], coll.simplex_vector()[i]));
    }
}

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
            CHECK(m.simplices_are_equal(e[i], expected_edge_simplices[i]));
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

        CHECK(m.simplices_are_equal(irtb_s, coll_s));
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
        CHECK(m.simplices_are_equal(itrb_collection.simplex_vector()[i], coll.simplex_vector()[i]));
    }
}

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
            CHECK((faces(m, e).contains(v) || m.simplices_are_equal(v, Simplex::vertex(center))));
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
            CHECK((faces(m, e).contains(v) || m.simplices_are_equal(v, Simplex::vertex(center))));
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
                (m.simplices_are_equal(Simplex::edge(t), e) ||
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
                (m.simplices_are_equal(Simplex::edge(t), e) ||
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
            CHECK(m.simplices_are_equal(Simplex::face(t), Simplex::face(e.tuple())));
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
        CHECK(m.simplices_are_equal(itrb_collection.simplex_vector()[i], coll.simplex_vector()[i]));
    }
}

TEST_CASE("simplex_link", "[simplex_collection][2D]")
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
            CHECK(m.simplices_are_equal(v, Simplex::vertex(center)));
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
            CHECK(m.simplices_are_equal(v, Simplex::vertex(center)));
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
        CHECK(m.simplices_are_equal(itrb_collection.simplex_vector()[i], coll.simplex_vector()[i]));
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
        std::set<long> target_vids({0, 3, 1, 5, 7, 8});
        std::set<long> vids;
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

TEST_CASE("simplex_faces_single_dimension", "[simplex_collection]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    SECTION("vertices_0123")
    {
        const std::vector<long> expected_vids{0, 1, 2, 3};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(0, 1, 0));
        for (long dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension(
                m,
                Simplex(get_primitive_type_from_id(dim), t),
                PrimitiveType::Vertex);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 2);
        CHECK(single_dim_faces[2].size() == 3);
        CHECK(single_dim_faces[3].size() == 4);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                CHECK(m.id(Simplex::vertex(vs[i])) == expected_vids[i]);
            }
        }
    }
    SECTION("vertices_1023")
    {
        const std::vector<long> expected_vids{1, 0, 2, 3};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(1, 0, 0));
        for (long dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension(
                m,
                Simplex(get_primitive_type_from_id(dim), t),
                PrimitiveType::Vertex);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 2);
        CHECK(single_dim_faces[2].size() == 3);
        CHECK(single_dim_faces[3].size() == 4);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                CHECK(m.id(Simplex::vertex(vs[i])) == expected_vids[i]);
            }
        }
    }
    SECTION("edges_0123")
    {
        const std::vector<std::array<long, 2>>
            expected_vids{{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(0, 1, 0));
        for (long dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension(
                m,
                Simplex(get_primitive_type_from_id(dim), t),
                PrimitiveType::Edge);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 0);
        CHECK(single_dim_faces[2].size() == 3);
        CHECK(single_dim_faces[3].size() == 6);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                const std::vector<Tuple> edge_vertices =
                    faces_single_dimension(m, Simplex::edge(vs[i]), PrimitiveType::Vertex);
                CHECK(edge_vertices.size() == 2);
                for (size_t j = 0; j < edge_vertices.size(); ++j) {
                    const long ev = m.id(Simplex::vertex(edge_vertices[j]));
                    CHECK(ev == expected_vids[i][j]);
                }
            }
        }
    }
    SECTION("faces_0123")
    {
        const std::vector<std::array<long, 3>> expected_vids{
            {0, 1, 2},
            {0, 3, 1},
            {1, 3, 2},
            {2, 3, 0}};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(0, 1, 0));
        for (long dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension(
                m,
                Simplex(get_primitive_type_from_id(dim), t),
                PrimitiveType::Face);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 0);
        CHECK(single_dim_faces[2].size() == 0);
        CHECK(single_dim_faces[3].size() == 4);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                const std::vector<Tuple> edge_vertices =
                    faces_single_dimension(m, Simplex::face(vs[i]), PrimitiveType::Vertex);
                CHECK(edge_vertices.size() == 3);
                for (size_t j = 0; j < edge_vertices.size(); ++j) {
                    const long ev = m.id(Simplex::vertex(edge_vertices[j]));
                    CHECK(ev == expected_vids[i][j]);
                }
            }
        }
    }
}

TEST_CASE("simplex_compare_faces_with_faces_single_dimension", "[simplex_collection]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    for (long cell_dim = 0; cell_dim < 4; ++cell_dim) {
        const PrimitiveType cell_type = get_primitive_type_from_id(cell_dim);

        const std::vector<Tuple> cells = m.get_all(cell_type);

        for (long face_dim = 0; face_dim < 4; ++face_dim) {
            const PrimitiveType face_type = get_primitive_type_from_id(face_dim);

            for (const Tuple& cell : cells) {
                const Simplex cell_simplex = Simplex(cell_type, cell);
                const std::vector<Tuple> fsd_vec =
                    faces_single_dimension(m, cell_simplex, face_type);

                SimplexCollection face_collection(m);
                for (const Tuple& f : fsd_vec) {
                    face_collection.add(Simplex(face_type, f));
                }
                face_collection.sort_and_clean();
                const std::vector<Simplex> faces_single_dim =
                    face_collection.simplex_vector(face_type);

                SimplexCollection bndry = faces(m, cell_simplex);
                const std::vector<Simplex> bndry_single_dim = bndry.simplex_vector(face_type);

                REQUIRE(faces_single_dim.size() == bndry_single_dim.size());
                for (size_t i = 0; i < faces_single_dim.size(); ++i) {
                    CHECK(m.simplices_are_equal(bndry_single_dim[i], faces_single_dim[i]));
                }
            }
        }
    }
}
