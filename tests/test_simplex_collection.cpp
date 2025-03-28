#include <array>
#include <catch2/catch_test_macros.hpp>
#include <set>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/IdSimplexCollection.hpp>
#include <wmtk/simplex/RawSimplex.hpp>
#include <wmtk/simplex/RawSimplexCollection.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/boundary.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/closed_star_iterable.hpp>
#include <wmtk/simplex/cofaces_in_simplex_iterable.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/cofaces_single_dimension_iterable.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/faces_iterable.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/half_closed_star_iterable.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/simplex/link_condition.hpp>
#include <wmtk/simplex/link_iterable.hpp>
#include <wmtk/simplex/link_single_dimension.hpp>
#include <wmtk/simplex/link_single_dimension_iterable.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/simplex/open_star_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/top_dimension_cofaces_iterable.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Stopwatch.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"
#include "tools/all_valid_local_tuples.hpp"

using namespace wmtk;
using namespace simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;
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
            const simplex::Simplex s0(m, PV, t);
            const simplex::Simplex s1(m, PV, m.switch_tuple(t, PE));
            CHECK(simplex::utils::SimplexComparisons::equal(m, s0, s1));
            if (m.is_boundary_edge(t)) {
                continue;
            }
            const simplex::Simplex s2(m, PV, m.switch_tuple(t, PF));
            CHECK(simplex::utils::SimplexComparisons::equal(m, s0, s2));
            CHECK(simplex::utils::SimplexComparisons::equal(m, s1, s2));
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PE);
        REQUIRE(edges.size() == 5);
        for (const Tuple& t : edges) {
            const simplex::Simplex s0(m, PE, t);
            const simplex::Simplex s1(m, PE, m.switch_tuple(t, PV));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s0, s1));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s1, s0));
            if (m.is_boundary_edge(t)) {
                continue;
            }
            const simplex::Simplex s2(m, PE, m.switch_tuple(t, PF));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s0, s2));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s2, s0));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s1, s2));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s2, s1));
        }
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PF);
        REQUIRE(faces.size() == 2);
        for (const Tuple& t : faces) {
            const simplex::Simplex s0(m, PF, t);
            const simplex::Simplex s1(m, PF, m.switch_tuple(t, PV));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s0, s1));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s1, s0));
            const simplex::Simplex s2(m, PF, m.switch_tuple(t, PE));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s0, s2));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s2, s0));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s1, s2));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s2, s1));
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
        simplex_collection.add(simplex::Simplex::vertex(m, t));
    }
    for (const auto& t : edges) {
        simplex_collection.add(simplex::Simplex::edge(m, t));
    }
    for (const auto& t : faces) {
        simplex_collection.add(simplex::Simplex::face(m, t));
    }
    REQUIRE(simplex_collection.simplex_vector().size() == 11);

    // test sorting and clean-up
    for (const auto& t : vertices) {
        simplex_collection.add(simplex::Simplex::vertex(m, t));
        break;
    }
    REQUIRE(simplex_collection.simplex_vector().size() == 12);
    simplex_collection.sort_and_clean();
    REQUIRE(simplex_collection.simplex_vector().size() == 11);
}

TEST_CASE("id_simplex_collection", "[id_simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::quad();
    const std::vector<Tuple> vertices = m.get_all(PV);
    REQUIRE(vertices.size() == 4);
    const std::vector<Tuple> edges = m.get_all(PE);
    REQUIRE(edges.size() == 5);
    const std::vector<Tuple> faces = m.get_all(PF);
    REQUIRE(faces.size() == 2);

    IdSimplexCollection simplex_collection(m);
    for (const auto& t : vertices) {
        simplex_collection.add(m.get_id_simplex(t, PrimitiveType::Vertex));
    }
    for (const auto& t : edges) {
        simplex_collection.add(m.get_id_simplex(t, PrimitiveType::Edge));
    }
    for (const auto& t : faces) {
        simplex_collection.add(m.get_id_simplex(t, PrimitiveType::Triangle));
    }
    REQUIRE(simplex_collection.simplex_vector().size() == 11);
    CHECK(simplex_collection.simplex_vector(PrimitiveType::Vertex).size() == 4);
    CHECK(simplex_collection.simplex_vector(PrimitiveType::Edge).size() == 5);
    CHECK(simplex_collection.simplex_vector(PrimitiveType::Triangle).size() == 2);
    CHECK(simplex_collection.simplex_vector_tuples(PrimitiveType::Vertex).size() == 4);
    CHECK(simplex_collection.simplex_vector_tuples(PrimitiveType::Edge).size() == 5);
    CHECK(simplex_collection.simplex_vector_tuples(PrimitiveType::Triangle).size() == 2);

    // test sorting and clean-up
    IdSimplex v0 = m.get_id_simplex(m.vertex_tuple_from_id(0), PrimitiveType::Vertex);
    simplex_collection.add(v0);
    REQUIRE(simplex_collection.simplex_vector().size() == 12);
    simplex_collection.sort_and_clean();
    REQUIRE(simplex_collection.simplex_vector().size() == 11);

    CHECK(simplex_collection.contains(v0));

    {
        IdSimplexCollection sc2(m);
        sc2.add(simplex_collection);
        CHECK(simplex_collection.simplex_vector().size() == 11);

        IdSimplexCollection sc_union = IdSimplexCollection::get_union(simplex_collection, sc2);
        IdSimplexCollection sc_intersection =
            IdSimplexCollection::get_intersection(simplex_collection, sc2);

        CHECK(sc_union.size() == simplex_collection.size());
        CHECK(sc_intersection.size() == simplex_collection.size());
        CHECK(sc_union == sc_intersection);
    }
}

TEST_CASE("simplex_faces", "[simplex_collection][2D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    const Tuple t = m.switch_face(m.edge_tuple_with_vs_and_t(0, 1, 0));

    SECTION("vertex")
    {
        SimplexCollection bd = faces(m, Simplex::vertex(m, t));
        REQUIRE(bd.simplex_vector().size() == 0);
    }
    SECTION("edge")
    {
        SimplexCollection bd = faces(m, Simplex::edge(m, t));
        REQUIRE(bd.simplex_vector().size() == 2);
        const std::vector<Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        REQUIRE(v.size() == 2);
        CHECK(m.id(v[0]) == 0);
        CHECK(m.id(v[1]) == 1);

        CHECK(bd.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("face")
    {
        SimplexCollection bd = faces(m, Simplex::face(m, t));
        REQUIRE(bd.simplex_vector().size() == 6);
        const std::vector<Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        CHECK(v.size() == 3);
        for (size_t i = 0; i < v.size(); ++i) {
            CHECK(m.id(v[i]) == i);
        }

        const std::vector<Simplex> e = bd.simplex_vector(PrimitiveType::Edge);
        CHECK(e.size() == 3);
        SimplexCollection expected_edges(m);
        expected_edges.add(Simplex::edge(m, m.edge_tuple_with_vs_and_t(0, 1, 0)));
        expected_edges.add(Simplex::edge(m, m.edge_tuple_with_vs_and_t(1, 2, 0)));
        expected_edges.add(Simplex::edge(m, m.edge_tuple_with_vs_and_t(2, 0, 0)));
        expected_edges.sort_and_clean();
        const std::vector<Simplex> expected_edge_simplices =
            expected_edges.simplex_vector(PrimitiveType::Edge);
        REQUIRE(e.size() <= 3);
        for (size_t i = 0; i < e.size(); ++i) {
            CHECK(simplex::utils::SimplexComparisons::equal(m, e[i], expected_edge_simplices[i]));
        }

        CHECK(bd.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("tetrahedron")
    {
        SimplexCollection bd = faces(m, Simplex::tetrahedron(m, t));
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


        const std::vector<Simplex> f = bd.simplex_vector(PrimitiveType::Triangle);
        CHECK(f.size() == 4);
        for (size_t i = 0; i < f.size(); ++i) {
            CHECK(m.id(f[i]) == i);
        }
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
}

TEST_CASE("simplex_faces_iterable", "[simplex_collection][iterable][2D]")
{
    tests::DEBUG_TriMesh m = tests::single_triangle();

    std::unique_ptr<Simplex> simplex;

    const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);

    SECTION("vertex")
    {
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge")
    {
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("face")
    {
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }

    FacesIterable itrb = faces_iterable(m, *simplex);
    SimplexCollection coll = faces(m, *simplex);

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

TEST_CASE("simplex_boundary", "[simplex_collection][2D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    const Tuple t = m.switch_face(m.edge_tuple_with_vs_and_t(0, 1, 0));

    SECTION("vertex")
    {
        SimplexCollection bd = boundary(m, Simplex::vertex(m, t));
        REQUIRE(bd.simplex_vector().size() == 0);
    }
    SECTION("edge")
    {
        SimplexCollection bd = boundary(m, Simplex::edge(m, t));
        REQUIRE(bd.simplex_vector().size() == 2);
        const std::vector<Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        REQUIRE(v.size() == 2);
        CHECK(m.id(v[0]) == 0);
        CHECK(m.id(v[1]) == 1);

        CHECK(bd.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("face")
    {
        SimplexCollection bd = boundary(m, Simplex::face(m, t));
        REQUIRE(bd.simplex_vector().size() == 3);
        CHECK(bd.simplex_vector(PrimitiveType::Vertex).size() == 0);


        const std::vector<Simplex> e = bd.simplex_vector(PrimitiveType::Edge);
        CHECK(e.size() == 3);
        SimplexCollection expected_edges(m);
        expected_edges.add(Simplex::edge(m, m.edge_tuple_with_vs_and_t(0, 1, 0)));
        expected_edges.add(Simplex::edge(m, m.edge_tuple_with_vs_and_t(1, 2, 0)));
        expected_edges.add(Simplex::edge(m, m.edge_tuple_with_vs_and_t(2, 0, 0)));
        expected_edges.sort_and_clean();
        const std::vector<Simplex> expected_edge_simplices =
            expected_edges.simplex_vector(PrimitiveType::Edge);
        REQUIRE(e.size() <= 3);
        for (size_t i = 0; i < e.size(); ++i) {
            CHECK(simplex::utils::SimplexComparisons::equal(m, e[i], expected_edge_simplices[i]));
        }

        CHECK(bd.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("tetrahedron")
    {
        SimplexCollection bd = boundary(m, Simplex::tetrahedron(m, t));
        REQUIRE(bd.simplex_vector().size() == 4);
        CHECK(bd.simplex_vector(PrimitiveType::Vertex).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Edge).size() == 0);


        const std::vector<Simplex> f = bd.simplex_vector(PrimitiveType::Triangle);
        REQUIRE(f.size() == 4);
        CHECK(m.id(f[0]) == 0);
        CHECK(m.id(f[1]) == 1);
        CHECK(m.id(f[2]) == 2);
        CHECK(m.id(f[3]) == 3);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
}

TEST_CASE("simplex_top_dimension_cofaces_tri", "[simplex_collection]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        const simplex::Simplex input = simplex::Simplex::vertex(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 6);
        REQUIRE(cc.simplex_vector(PrimitiveType::Triangle).size() == 6);

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

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Triangle));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Triangle, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("vertex_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 4, 0);
        Simplex input = simplex::Simplex::vertex(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Triangle).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 5);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Triangle));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Triangle, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("vertex_boundary_2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 0, 0);
        Simplex input = simplex::Simplex::vertex(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Triangle).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 5);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Triangle));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Triangle, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("vertex_boundary_3")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);
        Simplex input = simplex::Simplex::vertex(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Triangle).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 5);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Triangle));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Triangle, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        Simplex input = simplex::Simplex::edge(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Triangle).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 2);
        CHECK(m.id(cells[1]) == 7);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Triangle));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Triangle, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);

        Simplex input = simplex::Simplex::edge(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 1);
        REQUIRE(cc.simplex_vector(PrimitiveType::Triangle).size() == 1);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 5);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Triangle));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Triangle, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        Simplex input = simplex::Simplex::face(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 1);
        REQUIRE(cc.simplex_vector(PrimitiveType::Triangle).size() == 1);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 2);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Triangle));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Triangle, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
}

TEST_CASE("simplex_top_dimension_cofaces_tet", "[simplex_collection]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::six_cycle_tets();

    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);
        Simplex input = simplex::Simplex::vertex(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Tetrahedron).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 1);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Tetrahedron));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Tetrahedron, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 3, 0);

        Simplex input = simplex::Simplex::edge(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 6);
        REQUIRE(cc.simplex_vector(PrimitiveType::Tetrahedron).size() == 6);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 1);
        CHECK(m.id(cells[2]) == 2);
        CHECK(m.id(cells[3]) == 3);
        CHECK(m.id(cells[4]) == 4);
        CHECK(m.id(cells[5]) == 5);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Tetrahedron));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Tetrahedron, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 2, 0);

        Simplex input = simplex::Simplex::edge(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Tetrahedron).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 1);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Tetrahedron));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Tetrahedron, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("face_interior")
    {
        const Tuple t = m.face_tuple_from_vids(0, 2, 3);

        Simplex input = simplex::Simplex::face(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 2);
        REQUIRE(cc.simplex_vector(PrimitiveType::Tetrahedron).size() == 2);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        CHECK(m.id(cells[1]) == 1);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Tetrahedron));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Tetrahedron, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
    SECTION("face_boundary")
    {
        const Tuple t = m.face_tuple_from_vids(0, 1, 2);

        Simplex input = simplex::Simplex::face(m, t);
        SimplexCollection cc = top_dimension_cofaces(m, input);

        REQUIRE(cc.simplex_vector().size() == 1);
        REQUIRE(cc.simplex_vector(PrimitiveType::Tetrahedron).size() == 1);

        const auto& cells = cc.simplex_vector();
        CHECK(m.id(cells[0]) == 0);
        for (const Simplex& s : cells) {
            check_match_below_simplex_type(m, input, s);
        }

        SimplexCollection cc2(
            m,
            simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                m,
                top_dimension_cofaces_tuples(m, input),
                PrimitiveType::Tetrahedron));
        cc2.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc2));

        std::vector<Tuple> cc3_tuples;
        top_dimension_cofaces_tuples(m, input, cc3_tuples);
        SimplexCollection cc3(m);
        cc3.add(PrimitiveType::Tetrahedron, cc3_tuples);
        cc3.sort_and_clean();

        CHECK(SimplexCollection::are_simplex_collections_equal(cc, cc3));
    }
}

TEST_CASE("simplex_top_dimension_cofaces_tri_iterable", "[simplex_collection][iterable][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    std::unique_ptr<Simplex> simplex;

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 4, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 0, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_3")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(6, 5, 4);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_4")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(1, 4, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }

    TopDimensionCofacesIterable itrb = top_dimension_cofaces_iterable(m, *simplex);
    SimplexCollection coll = top_dimension_cofaces(m, *simplex);

    SimplexCollection itrb_collection(m);
    for (const Tuple& t : itrb) {
        itrb_collection.add(simplex::Simplex(m, m.top_simplex_type(), t));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        check_match_below_simplex_type(m, *simplex, coll_s);

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_top_dimension_cofaces_tet_iterable", "[simplex_collection][iterable][3D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::six_cycle_tets();

    std::unique_ptr<Simplex> simplex;

    SECTION("vertex_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 0, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_3")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 3, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 3, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 2, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary_2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("face_interior")
    {
        const Tuple t = m.face_tuple_from_vids(0, 2, 3);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }
    SECTION("face_boundary")
    {
        const Tuple t = m.face_tuple_from_vids(0, 1, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }

    TopDimensionCofacesIterable itrb = top_dimension_cofaces_iterable(m, *simplex);
    SimplexCollection coll = top_dimension_cofaces(m, *simplex);

    SimplexCollection itrb_collection(m);
    for (const Tuple& t : itrb) {
        itrb_collection.add(simplex::Simplex(m, m.top_simplex_type(), t));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        check_match_below_simplex_type(m, *simplex, coll_s);

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_top_dimension_cofaces_edge_iterable", "[simplex_collection][iterable][1D]")
{
    tests::DEBUG_EdgeMesh m = tests::two_segments();

    std::unique_ptr<Simplex> simplex;

    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_from_vids(0, 1);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_from_vids(1, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge")
    {
        const Tuple t = m.edge_tuple_from_vids(0, 1);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }

    TopDimensionCofacesIterable itrb = top_dimension_cofaces_iterable(m, *simplex);
    SimplexCollection coll = top_dimension_cofaces(m, *simplex);

    SimplexCollection itrb_collection(m);
    for (const Tuple& t : itrb) {
        itrb_collection.add(simplex::Simplex(m, m.top_simplex_type(), t));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        check_match_below_simplex_type(m, *simplex, coll_s);

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_top_dimension_cofaces_performance", "[simplex_collection][iterable][3D][.]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::two_by_two_by_two_grids_tets();

    std::unique_ptr<Simplex> simplex;


    const auto vertex_tuples = m.get_all(PrimitiveType::Vertex);
    int64_t counter = 0;
    {
        wmtk::utils::StopWatch sw("TDC_ITRBL");
        for (size_t i = 0; i < 100000; ++i) {
            for (const Tuple& t : vertex_tuples) {
                const simplex::Simplex v(m, PrimitiveType::Vertex, t);
                for (const Tuple& tt : top_dimension_cofaces_iterable(m, v)) {
                    ++counter;
                }
            }
        }
    }
    logger().info("Counter = {}", counter);
}

TEST_CASE("simplex_open_star", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection os = open_star(m, simplex::Simplex::vertex(m, t));

        REQUIRE(os.simplex_vector().size() == 13);
        CHECK(os.simplex_vector(PrimitiveType::Triangle).size() == 6);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 6);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 1);

        const auto& simplices = os.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(m, t);
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
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 4, 0);

        SimplexCollection os = open_star(m, simplex::Simplex::vertex(m, t));

        REQUIRE(os.simplex_vector().size() == 6);
        CHECK(os.simplex_vector(PrimitiveType::Triangle).size() == 2);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 3);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 1);

        const auto& simplices = os.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(m, t);
        CHECK(m.id(simplices[0]) == m.id(v));

        CHECK(faces(m, simplices[1]).contains(v));
        CHECK(faces(m, simplices[2]).contains(v));
        CHECK(faces(m, simplices[3]).contains(v));

        CHECK(m.id(simplices[4]) == 0);
        CHECK(m.id(simplices[5]) == 5);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection os = open_star(m, simplex::Simplex::edge(m, t));

        REQUIRE(os.simplex_vector().size() == 3);
        CHECK(os.simplex_vector(PrimitiveType::Triangle).size() == 2);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 0);

        const auto& simplices = os.simplex_vector();

        CHECK(m.id(simplices[0]) == m.id(simplex::Simplex::edge(m, t)));

        CHECK(m.id(simplices[1]) == 2);
        CHECK(m.id(simplices[2]) == 7);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);

        SimplexCollection os = open_star(m, simplex::Simplex::edge(m, t));

        REQUIRE(os.simplex_vector().size() == 2);
        CHECK(os.simplex_vector(PrimitiveType::Triangle).size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 0);

        const auto& simplices = os.simplex_vector();

        CHECK(m.id(simplices[0]) == m.id(simplex::Simplex::edge(m, t)));

        CHECK(m.id(simplices[1]) == 5);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection os = open_star(m, simplex::Simplex::face(m, t));

        REQUIRE(os.simplex_vector().size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Triangle).size() == 1);
        CHECK(os.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(os.simplex_vector(PrimitiveType::Vertex).size() == 0);

        const auto& simplices = os.simplex_vector();
        CHECK(m.id(simplices[0]) == 2);
    }
}

TEST_CASE("simplex_open_star_trimesh", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::vertex(m, t));
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::vertex(m, t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 4, 0);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::vertex(m, t));
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::vertex(m, t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::edge(m, t));
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::edge(m, t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::edge(m, t));
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::edge(m, t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection os_tri = open_star(m, simplex::Simplex::face(m, t));
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::face(m, t));

        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
}

TEST_CASE("simplex_open_star_tri_iterable", "[simplex_collection][iterable][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    std::unique_ptr<Simplex> simplex;

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 4, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 0, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_3")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(6, 5, 4);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_4")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(1, 4, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }

    OpenStarIterable itrb = open_star_iterable(m, *simplex);
    SimplexCollection coll = open_star_slow(m, *simplex);

    SimplexCollection itrb_collection(m);
    for (const simplex::IdSimplex& s : itrb) {
        itrb_collection.add(m.get_simplex(s));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_open_star_tet_iterable", "[simplex_collection][iterable][3D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::six_cycle_tets();

    std::unique_ptr<Simplex> simplex;

    SECTION("vertex_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 0, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_3")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 3, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 3, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 2, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary_2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("face_interior")
    {
        const Tuple t = m.face_tuple_from_vids(0, 2, 3);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }
    SECTION("face_boundary")
    {
        const Tuple t = m.face_tuple_from_vids(0, 1, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }
    SECTION("tet")
    {
        const Tuple t = m.tet_tuple_from_vids(0, 1, 2, 3);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Tetrahedron, t);
    }

    OpenStarIterable itrb = open_star_iterable(m, *simplex);
    SimplexCollection coll = open_star_slow(m, *simplex);

    SimplexCollection itrb_collection(m);
    for (const simplex::IdSimplex& s : itrb) {
        itrb_collection.add(m.get_simplex(s));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_closed_star", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection cs = closed_star(m, Simplex::vertex(m, t));

        REQUIRE(cs.simplex_vector().size() == 25);
        CHECK(cs.simplex_vector(PrimitiveType::Triangle).size() == 6);
        CHECK(cs.simplex_vector(PrimitiveType::Edge).size() == 12);
        CHECK(cs.simplex_vector(PrimitiveType::Vertex).size() == 7);

        const auto& simplices = cs.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(m, t);
        CHECK(m.id(simplices[0]) == 0);
        CHECK(m.id(simplices[1]) == 1);
        CHECK(m.id(simplices[2]) == 3);
        CHECK(m.id(simplices[3]) == 4);
        CHECK(m.id(simplices[4]) == 5);
        CHECK(m.id(simplices[5]) == 7);
        CHECK(m.id(simplices[6]) == 8);

        for (size_t i = 7; i < 19; ++i) {
            const Simplex& e = simplices[i];
            const Tuple center = m.switch_tuples(e.tuple(), {PE, PV});
            CHECK(
                (faces(m, e).contains(v) ||
                 simplex::utils::SimplexComparisons::equal(m, v, Simplex::vertex(m, center))));
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
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 4, 0);

        SimplexCollection cs = closed_star(m, Simplex::vertex(m, t));

        REQUIRE(cs.simplex_vector().size() == 11);
        CHECK(cs.simplex_vector(PrimitiveType::Triangle).size() == 2);
        CHECK(cs.simplex_vector(PrimitiveType::Edge).size() == 5);
        CHECK(cs.simplex_vector(PrimitiveType::Vertex).size() == 4);

        const auto& simplices = cs.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(m, t);
        CHECK(m.id(simplices[0]) == 0);
        CHECK(m.id(simplices[1]) == 3);
        CHECK(m.id(simplices[2]) == 4);
        CHECK(m.id(simplices[3]) == 7);

        for (size_t i = 4; i < 9; ++i) {
            const Simplex& e = simplices[i];
            const Tuple center = m.switch_tuples(e.tuple(), {PE, PV});
            CHECK(
                (faces(m, e).contains(v) ||
                 simplex::utils::SimplexComparisons::equal(m, v, Simplex::vertex(m, center))));
        }

        CHECK(m.id(simplices[9]) == 0);
        CHECK(m.id(simplices[10]) == 5);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection cs = closed_star(m, Simplex::edge(m, t));

        REQUIRE(cs.simplex_vector().size() == 11);
        CHECK(cs.simplex_vector(PrimitiveType::Triangle).size() == 2);
        CHECK(cs.simplex_vector(PrimitiveType::Edge).size() == 5);
        CHECK(cs.simplex_vector(PrimitiveType::Vertex).size() == 4);

        const auto& simplices = cs.simplex_vector();

        CHECK(m.id(simplices[0]) == 1);
        CHECK(m.id(simplices[1]) == 4);
        CHECK(m.id(simplices[2]) == 5);
        CHECK(m.id(simplices[3]) == 8);

        SimplexCollection t_bd = faces(m, Simplex::edge(m, t));

        for (size_t i = 4; i < 9; ++i) {
            const Simplex& e = simplices[i];
            SimplexCollection e_bd = faces(m, e);
            SimplexCollection bd_intersection = SimplexCollection::get_intersection(e_bd, t_bd);
            CHECK(
                (simplex::utils::SimplexComparisons::equal(m, Simplex::edge(m, t), e) ||
                 bd_intersection.simplex_vector().size() == 1));
        }

        CHECK(m.id(simplices[9]) == 2);
        CHECK(m.id(simplices[10]) == 7);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);

        SimplexCollection cs = closed_star(m, Simplex::edge(m, t));

        REQUIRE(cs.simplex_vector().size() == 7);
        CHECK(cs.simplex_vector(PrimitiveType::Triangle).size() == 1);
        CHECK(cs.simplex_vector(PrimitiveType::Edge).size() == 3);
        CHECK(cs.simplex_vector(PrimitiveType::Vertex).size() == 3);

        const auto& simplices = cs.simplex_vector();

        CHECK(m.id(simplices[0]) == 3);
        CHECK(m.id(simplices[1]) == 4);
        CHECK(m.id(simplices[2]) == 7);

        SimplexCollection t_bd = faces(m, Simplex::edge(m, t));

        for (size_t i = 3; i < 6; ++i) {
            const Simplex& e = simplices[i];
            SimplexCollection e_bd = faces(m, e);
            SimplexCollection bd_intersection = SimplexCollection::get_intersection(e_bd, t_bd);
            CHECK(
                (simplex::utils::SimplexComparisons::equal(m, Simplex::edge(m, t), e) ||
                 bd_intersection.simplex_vector().size() == 1));
        }

        CHECK(m.id(simplices[6]) == 5);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection cs = closed_star(m, Simplex::face(m, t));

        REQUIRE(cs.simplex_vector().size() == 7);
        CHECK(cs.simplex_vector(PrimitiveType::Triangle).size() == 1);
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
                Simplex::face(m, t),
                Simplex::face(m, e.tuple())));
        }

        CHECK(m.id(simplices[6]) == 2);
    }
}

TEST_CASE("simplex_closed_star_tri_iterable", "[simplex_collection][iterable][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    std::unique_ptr<Simplex> simplex;

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 4, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 0, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_3")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(6, 5, 4);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_4")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(1, 4, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }

    ClosedStarIterable itrb = closed_star_iterable(m, *simplex);
    SimplexCollection coll = closed_star(m, *simplex);

    SimplexCollection itrb_collection(m);
    for (const simplex::IdSimplex& s : itrb) {
        itrb_collection.add(m.get_simplex(s));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_closed_star_tet_iterable", "[simplex_collection][iterable][3D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::six_cycle_tets();

    std::unique_ptr<Simplex> simplex;

    SECTION("vertex_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 0, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary_3")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 3, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 3, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 2, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary_2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("face_interior")
    {
        const Tuple t = m.face_tuple_from_vids(0, 2, 3);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }
    SECTION("face_boundary")
    {
        const Tuple t = m.face_tuple_from_vids(0, 1, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }
    SECTION("tet")
    {
        const Tuple t = m.tet_tuple_from_vids(0, 1, 2, 3);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Tetrahedron, t);
    }

    ClosedStarIterable itrb = closed_star_iterable(m, *simplex);
    SimplexCollection coll = closed_star(m, *simplex);

    SimplexCollection itrb_collection(m);
    for (const simplex::IdSimplex& s : itrb) {
        itrb_collection.add(m.get_simplex(s));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_closed_star_edge_iterable", "[simplex_collection][iterable][1D]")
{
    tests::DEBUG_EdgeMesh m = tests::two_segments();

    std::unique_ptr<Simplex> simplex;

    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_from_vids(0, 1);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_from_vids(1, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge")
    {
        const Tuple t = m.edge_tuple_from_vids(0, 1);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }

    ClosedStarIterable itrb = closed_star_iterable(m, *simplex);
    SimplexCollection coll = closed_star(m, *simplex);

    SimplexCollection itrb_collection(m);
    for (const simplex::IdSimplex& s : itrb) {
        itrb_collection.add(m.get_simplex(s));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_half_closed_star_tri_iterable", "[simplex_collection][iterable][3D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    std::unique_ptr<Simplex> simplex;

    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }

    const simplex::SimplexCollection vertex_open_star =
        simplex::open_star(m, simplex::Simplex::vertex(m, simplex->tuple()));
    const simplex::SimplexCollection edge_closed_star =
        simplex::closed_star(m, simplex::Simplex::edge(m, simplex->tuple()));
    const simplex::SimplexCollection coll =
        simplex::SimplexCollection::get_intersection(vertex_open_star, edge_closed_star);

    HalfClosedStarIterable itrb = half_closed_star_iterable(m, simplex->tuple());

    SimplexCollection itrb_collection(m);
    for (const simplex::IdSimplex& s : itrb) {
        itrb_collection.add(m.get_simplex(s));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_half_closed_star_tet_iterable", "[simplex_collection][iterable][3D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::six_cycle_tets();

    std::unique_ptr<Simplex> simplex;

    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(2, 3, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary_1")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 2, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary_2")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }

    const simplex::SimplexCollection vertex_open_star =
        simplex::open_star(m, simplex::Simplex::vertex(m, simplex->tuple()));
    const simplex::SimplexCollection edge_closed_star =
        simplex::closed_star(m, simplex::Simplex::edge(m, simplex->tuple()));
    const simplex::SimplexCollection coll =
        simplex::SimplexCollection::get_intersection(vertex_open_star, edge_closed_star);

    HalfClosedStarIterable itrb = half_closed_star_iterable(m, simplex->tuple());

    SimplexCollection itrb_collection(m);
    for (const simplex::IdSimplex& s : itrb) {
        itrb_collection.add(m.get_simplex(s));
    }
    REQUIRE(itrb_collection.size() == coll.size());

    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.size(); ++i) {
        const Simplex& irtb_s = itrb_collection.simplex_vector()[i];
        const Simplex& coll_s = coll.simplex_vector()[i];

        CHECK(simplex::utils::SimplexComparisons::equal(m, irtb_s, coll_s));
    }
}

TEST_CASE("simplex_link_2d", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection sc = link(m, Simplex::vertex(m, t));

        REQUIRE(sc.simplex_vector().size() == 12);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 6);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 6);

        const auto& simplices = sc.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(m, t);
        CHECK(m.id(simplices[0]) == 0);
        CHECK(m.id(simplices[1]) == 1);
        CHECK(m.id(simplices[2]) == 3);
        CHECK(m.id(simplices[3]) == 5);
        CHECK(m.id(simplices[4]) == 7);
        CHECK(m.id(simplices[5]) == 8);

        for (size_t i = 6; i < 12; ++i) {
            const Simplex& e = simplices[i];
            const Tuple center = m.switch_tuples(e.tuple(), {PE, PV});
            CHECK(simplex::utils::SimplexComparisons::equal(m, v, Simplex::vertex(m, center)));
        }
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 4, 0);

        SimplexCollection sc = link(m, Simplex::vertex(m, t));

        REQUIRE(sc.simplex_vector().size() == 5);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 2);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 3);

        const auto& simplices = sc.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(m, t);
        CHECK(m.id(simplices[0]) == 0);
        CHECK(m.id(simplices[1]) == 4);
        CHECK(m.id(simplices[2]) == 7);

        for (size_t i = 3; i < 5; ++i) {
            const Simplex& e = simplices[i];
            const Tuple center = m.switch_tuples(e.tuple(), {PE, PV});
            CHECK(simplex::utils::SimplexComparisons::equal(m, v, Simplex::vertex(m, center)));
        }
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection sc = link(m, Simplex::edge(m, t));

        REQUIRE(sc.simplex_vector().size() == 2);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 2);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 1);
        CHECK(m.id(simplices[1]) == 8);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);

        SimplexCollection sc = link(m, Simplex::edge(m, t));

        REQUIRE(sc.simplex_vector().size() == 1);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 1);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 4);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);

        SimplexCollection cs = link(m, Simplex::face(m, t));

        REQUIRE(cs.simplex_vector().size() == 0);
    }
}

TEST_CASE("simplex_link_tri_iterable", "[simplex_collection][iterable][2D]")
{
    logger().set_level(spdlog::level::err);

    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    std::unique_ptr<Simplex> simplex;

    SECTION("vertex_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<simplex::Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 4, 0);
        simplex = std::make_unique<simplex::Simplex>(m, PrimitiveType::Vertex, t);
    }
    SECTION("edge_interior")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("edge_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(3, 7, 5);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Edge, t);
    }
    SECTION("face")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        simplex = std::make_unique<Simplex>(m, PrimitiveType::Triangle, t);
    }

    LinkIterable itrb = link_iterable(m, *simplex);
    SimplexCollection coll = link_slow(m, *simplex);

    SimplexCollection itrb_collection(m);
    for (const simplex::IdSimplex& s : itrb) {
        itrb_collection.add(m.get_simplex(s));
    }
    REQUIRE(itrb_collection.size() == coll.size());
    itrb_collection.sort_and_clean();
    REQUIRE(itrb_collection.size() == coll.size());

    for (size_t i = 0; i < coll.simplex_vector().size(); ++i) {
        CHECK(simplex::utils::SimplexComparisons::equal(
            m,
            itrb_collection.simplex_vector()[i],
            coll.simplex_vector()[i]));
    }
}

TEST_CASE("simplex_link_tet_iterable", "[simplex_collection][iterable][3D]")
{
    logger().set_level(spdlog::level::err);

    auto mp = std::make_unique<TetMesh>(tests_3d::six_cycle_tets());
    Mesh& m = *mp;

    auto compare_collections = [&m](const simplex::Simplex& s) {
        SimplexCollection comp = link_slow(m, s);

        auto itrb = link_iterable(m, s);
        SimplexCollection itrb_collection(m);
        for (const simplex::IdSimplex& s : itrb) {
            itrb_collection.add(m.get_simplex(s));
        }
        REQUIRE(comp.size() == itrb_collection.size());
        itrb_collection.sort_and_clean();
        REQUIRE(comp.size() == itrb_collection.size());

        for (size_t i = 0; i < itrb_collection.simplex_vector().size(); ++i) {
            CHECK(simplex::utils::SimplexComparisons::equal(
                m,
                itrb_collection.simplex_vector()[i],
                comp.simplex_vector()[i]));
        }
    };

    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        const simplex::Simplex s(m, PrimitiveType::Vertex, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        const simplex::Simplex s(m, PrimitiveType::Edge, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
        const simplex::Simplex s(m, PrimitiveType::Triangle, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Tetrahedron)) {
        const simplex::Simplex s(m, PrimitiveType::Tetrahedron, t);
        compare_collections(s);
    }
}

TEST_CASE("simplex_open_star_tetmesh", "[simplex_collection]")
{
    auto mp = std::make_unique<TetMesh>(tests_3d::two_by_two_by_two_grids_tets());
    Mesh& m = *mp;

    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        SimplexCollection os_tri = open_star(m, simplex::Simplex::vertex(m, t));
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::vertex(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        SimplexCollection os_tri = open_star(m, simplex::Simplex::edge(m, t));
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::edge(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
        SimplexCollection os_tri = open_star(m, simplex::Simplex::face(m, t));
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::face(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Tetrahedron)) {
        SimplexCollection os_tri = open_star(m, simplex::Simplex::tetrahedron(m, t));
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::tetrahedron(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));
    }
}

TEST_CASE("simplex_link_trimesh", "[simplex_collection]")
{
    auto mp = std::make_unique<TriMesh>(tests::hex_plus_two());
    Mesh& m = *mp;

    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        SimplexCollection os_tri = link(m, simplex::Simplex::vertex(m, t));
        SimplexCollection os_m = link_slow(m, simplex::Simplex::vertex(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(m.top_simplex_type())) {
            SimplexCollection single_dim =
                link_single_dimension(m, simplex::Simplex::vertex(m, t), pt);
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));
            CHECK(SimplexCollection::are_simplex_collections_equal(single_dim, single_dim_comp));
        }
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        SimplexCollection os_tri = link(m, simplex::Simplex::edge(m, t));
        SimplexCollection os_m = link_slow(m, simplex::Simplex::edge(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(m.top_simplex_type())) {
            SimplexCollection single_dim =
                link_single_dimension(m, simplex::Simplex::edge(m, t), pt);
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));
            CHECK(SimplexCollection::are_simplex_collections_equal(single_dim, single_dim_comp));
        }
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
        SimplexCollection os_tri = link(m, simplex::Simplex::face(m, t));
        SimplexCollection os_m = link_slow(m, simplex::Simplex::face(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(m.top_simplex_type())) {
            SimplexCollection single_dim =
                link_single_dimension(m, simplex::Simplex::face(m, t), pt);
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));
            CHECK(SimplexCollection::are_simplex_collections_equal(single_dim, single_dim_comp));
        }
    }
}

TEST_CASE("simplex_link_tetmesh", "[simplex_collection]")
{
    auto mp = std::make_unique<TetMesh>(tests_3d::two_by_two_by_two_grids_tets());
    Mesh& m = *mp;

    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        SimplexCollection os_tri = link(m, simplex::Simplex::vertex(m, t));
        SimplexCollection os_m = link_slow(m, simplex::Simplex::vertex(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(m.top_simplex_type())) {
            SimplexCollection single_dim =
                link_single_dimension(m, simplex::Simplex::vertex(m, t), pt);
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));
            CHECK(SimplexCollection::are_simplex_collections_equal(single_dim, single_dim_comp));
        }
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        SimplexCollection os_tri = link(m, simplex::Simplex::edge(m, t));
        SimplexCollection os_m = link_slow(m, simplex::Simplex::edge(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(m.top_simplex_type())) {
            SimplexCollection single_dim =
                link_single_dimension(m, simplex::Simplex::edge(m, t), pt);
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));
            CHECK(SimplexCollection::are_simplex_collections_equal(single_dim, single_dim_comp));
        }
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
        SimplexCollection os_tri = link(m, simplex::Simplex::face(m, t));
        SimplexCollection os_m = link_slow(m, simplex::Simplex::face(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(m.top_simplex_type())) {
            SimplexCollection single_dim =
                link_single_dimension(m, simplex::Simplex::face(m, t), pt);
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));
            CHECK(SimplexCollection::are_simplex_collections_equal(single_dim, single_dim_comp));
        }
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Tetrahedron)) {
        SimplexCollection os_tri = link(m, simplex::Simplex::tetrahedron(m, t));
        SimplexCollection os_m = link_slow(m, simplex::Simplex::tetrahedron(m, t));
        CHECK(SimplexCollection::are_simplex_collections_equal(os_m, os_tri));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(m.top_simplex_type())) {
            SimplexCollection single_dim =
                link_single_dimension(m, simplex::Simplex::tetrahedron(m, t), pt);
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));
            CHECK(SimplexCollection::are_simplex_collections_equal(single_dim, single_dim_comp));
        }
    }
}

TEST_CASE("simplex_cofaces_in_simplex_iterable", "[simplex_collection][iterable]")
{
    SECTION("tri")
    {
        tests::DEBUG_TriMesh m = tests::hex_plus_two();
        const simplex::Simplex v(m, PrimitiveType::Vertex, m.vertex_tuple_from_id(0));
        const simplex::Simplex e(m, PrimitiveType::Edge, m.edge_tuple_from_vids(0, 1));
        const simplex::Simplex f(m, PrimitiveType::Triangle, m.tuple_from_face_id(0));
        // vertex - tri
        {
            SimplexCollection collection(m);
            for (const Tuple& t : cofaces_in_simplex_iterable(m, v, PrimitiveType::Triangle)) {
                collection.add(PrimitiveType::Edge, t);
            }
            CHECK(collection.size() == 2);
            collection.sort_and_clean();
            CHECK(collection.size() == 2);
        }
        // vertex - edge
        {
            SimplexCollection collection(m);
            for (const Tuple& t : cofaces_in_simplex_iterable(m, v, PrimitiveType::Edge)) {
                collection.add(PrimitiveType::Edge, t);
            }
            CHECK(collection.size() == 1);
            collection.sort_and_clean();
            CHECK(collection.size() == 1);
        }
        // edge - tri
        {
            SimplexCollection collection(m);
            for (const Tuple& t : cofaces_in_simplex_iterable(m, e, PrimitiveType::Triangle)) {
                collection.add(PrimitiveType::Edge, t);
            }
            CHECK(collection.size() == 1);
            collection.sort_and_clean();
            CHECK(collection.size() == 1);
        }
        // tri - tri
        {
            SimplexCollection collection(m);
            for (const Tuple& t : cofaces_in_simplex_iterable(m, f, PrimitiveType::Triangle)) {
                collection.add(PrimitiveType::Edge, t);
            }
            CHECK(collection.size() == 1);
        }
        // tri - edge
        {
            SimplexCollection collection(m);
            for (const Tuple& t : cofaces_in_simplex_iterable(m, f, PrimitiveType::Edge)) {
                collection.add(PrimitiveType::Edge, t);
            }
            CHECK(collection.size() == 0);
        }
    }
    SECTION("tet")
    {
        tests_3d::DEBUG_TetMesh m = tests_3d::six_cycle_tets();
        const simplex::Simplex v(m, PrimitiveType::Vertex, m.vertex_tuple_from_id(0));
        const simplex::Simplex e(m, PrimitiveType::Edge, m.edge_tuple_from_vids(0, 1));
        const simplex::Simplex f(m, PrimitiveType::Triangle, m.face_tuple_from_vids(0, 1, 2));
        const simplex::Simplex tet(m, PrimitiveType::Triangle, m.tet_tuple_from_vids(0, 1, 2, 3));
        // vertex - tet
        {
            SimplexCollection collection(m);
            for (const Tuple& t : cofaces_in_simplex_iterable(m, v, PrimitiveType::Tetrahedron)) {
                collection.add(PrimitiveType::Edge, t);
                collection.add(PrimitiveType::Triangle, t);
            }
            CHECK(collection.size() == 6);
            collection.sort_and_clean();
            CHECK(collection.size() == 6);
            CHECK(collection.simplex_vector(PrimitiveType::Edge).size() == 3);
            CHECK(collection.simplex_vector(PrimitiveType::Triangle).size() == 3);
        }
        // edge - tet
        {
            SimplexCollection collection(m);
            for (const Tuple& t : cofaces_in_simplex_iterable(m, e, PrimitiveType::Tetrahedron)) {
                collection.add(PrimitiveType::Triangle, t);
            }
            CHECK(collection.size() == 2);
            collection.sort_and_clean();
            CHECK(collection.size() == 2);
        }
        // vertex - tri
        {
            SimplexCollection collection(m);
            for (const Tuple& t : cofaces_in_simplex_iterable(m, v, PrimitiveType::Triangle)) {
                collection.add(PrimitiveType::Edge, t);
            }
            CHECK(collection.size() == 2);
            collection.sort_and_clean();
            CHECK(collection.size() == 2);
        }
        // edge - tri
        {
            SimplexCollection collection(m);
            for (const Tuple& t : cofaces_in_simplex_iterable(m, e, PrimitiveType::Triangle)) {
                collection.add(PrimitiveType::Triangle, t);
            }
            CHECK(collection.size() == 1);
        }
    }
}

TEST_CASE("simplex_cofaces_single_dimension", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    Tuple t;
    std::vector<Tuple> tc;
    std::set<int64_t> target_vids;
    SECTION("vertex_interior")
    {
        t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        const simplex::Simplex input = simplex::Simplex::vertex(m, t);
        tc = cofaces_single_dimension_tuples(m, input, PrimitiveType::Edge);
        REQUIRE(tc.size() == 6);
        target_vids = std::set<int64_t>({0, 3, 1, 5, 7, 8});
    }

    SECTION("vertex_boundary")
    {
        t = m.edge_tuple_with_vs_and_t(3, 4, 0);
        const simplex::Simplex input = simplex::Simplex::vertex(m, t);
        tc = cofaces_single_dimension_tuples(m, input, PrimitiveType::Edge);
        REQUIRE(tc.size() == 3);
        target_vids = std::set<int64_t>({0, 4, 7});
    }

    SimplexCollection sc(
        m,
        simplex::utils::tuple_vector_to_homogeneous_simplex_vector(m, tc, PrimitiveType::Edge));
    sc.sort();


    // check the lower dimension coface is the same as input
    for (const Tuple& tup : tc) {
        CHECK(m.id(tup, PrimitiveType::Vertex) == m.id(t, PrimitiveType::Vertex));
    }

    const auto& cells = sc.simplex_vector();
    std::set<int64_t> vids;
    std::transform(
        cells.begin(),
        cells.end(),
        std::inserter(vids, vids.end()),
        [&](const Simplex& s) { return m.id(m.switch_vertex(s.tuple()), PrimitiveType::Vertex); });

    CHECK(target_vids == vids);

    // check the lower dimension coface is the same as input
    for (const Tuple& tup : tc) {
        CHECK(m.id(tup, PrimitiveType::Vertex) == m.id(t, PrimitiveType::Vertex));
    }
}

TEST_CASE("simplex_cofaces_single_dimension_tri", "[simplex_collection][2D]")
{
    auto mp = std::make_unique<TriMesh>(tests::hex_plus_two());
    Mesh& m = *mp;

    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::vertex(m, t));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(m.top_simplex_type())) {
            SimplexCollection single_dim_tri =
                cofaces_single_dimension(*mp, simplex::Simplex::vertex(m, t), pt);
            SimplexCollection single_dim_all =
                cofaces_single_dimension(m, simplex::Simplex::vertex(m, t), pt);
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));
            CHECK(
                SimplexCollection::are_simplex_collections_equal(single_dim_tri, single_dim_comp));
            CHECK(
                SimplexCollection::are_simplex_collections_equal(single_dim_all, single_dim_comp));
        }
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::edge(m, t));

        for (const PrimitiveType pt :
             wmtk::utils::primitive_range(PrimitiveType::Edge, m.top_simplex_type())) {
            SimplexCollection single_dim_tri =
                cofaces_single_dimension(*mp, simplex::Simplex::edge(m, t), pt);
            SimplexCollection single_dim_all =
                cofaces_single_dimension(m, simplex::Simplex::edge(m, t), pt);
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));
            CHECK(
                SimplexCollection::are_simplex_collections_equal(single_dim_tri, single_dim_comp));
            CHECK(
                SimplexCollection::are_simplex_collections_equal(single_dim_all, single_dim_comp));
        }
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
        SimplexCollection os_m = open_star_slow(m, simplex::Simplex::face(m, t));

        SimplexCollection single_dim_tri =
            cofaces_single_dimension(*mp, simplex::Simplex::face(m, t), PrimitiveType::Triangle);
        SimplexCollection single_dim_all =
            cofaces_single_dimension(m, simplex::Simplex::face(m, t), PrimitiveType::Triangle);
        SimplexCollection single_dim_comp(m, os_m.simplex_vector(PrimitiveType::Triangle));
        CHECK(SimplexCollection::are_simplex_collections_equal(single_dim_tri, single_dim_comp));
        CHECK(SimplexCollection::are_simplex_collections_equal(single_dim_all, single_dim_comp));
    }
}

TEST_CASE("simplex_cofaces_single_dimension_tri_iterable", "[simplex_collection][iterable][2D]")
{
    auto mp = std::make_unique<TriMesh>(tests::hex_plus_two());
    Mesh& m = *mp;

    auto compare_collections = [&m](const simplex::Simplex& s) {
        SimplexCollection os_m = open_star_slow(m, s);
        for (const PrimitiveType pt :
             wmtk::utils::primitive_range(s.primitive_type(), m.top_simplex_type())) {
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));

            auto itrb = cofaces_single_dimension_iterable(m, s, pt);
            SimplexCollection itrb_collection(m);
            for (const Tuple& tt : itrb) {
                itrb_collection.add(simplex::Simplex(m, pt, tt));
            }
            REQUIRE(single_dim_comp.size() == itrb_collection.size());
            itrb_collection.sort_and_clean();
            REQUIRE(single_dim_comp.size() == itrb_collection.size());

            for (size_t i = 0; i < itrb_collection.simplex_vector().size(); ++i) {
                CHECK(simplex::utils::SimplexComparisons::equal(
                    m,
                    itrb_collection.simplex_vector()[i],
                    single_dim_comp.simplex_vector()[i]));
            }
        }
    };

    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        const simplex::Simplex s(m, PrimitiveType::Vertex, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        const simplex::Simplex s(m, PrimitiveType::Edge, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
        const simplex::Simplex s(m, PrimitiveType::Triangle, t);
        compare_collections(s);
    }
}

TEST_CASE("simplex_cofaces_single_dimension_tet_iterable", "[simplex_collection][iterable][3D]")
{
    auto mp = std::make_unique<TetMesh>(tests_3d::six_cycle_tets());
    Mesh& m = *mp;

    auto compare_collections = [&m](const simplex::Simplex& s) {
        SimplexCollection os_m = open_star_slow(m, s);
        for (const PrimitiveType pt :
             wmtk::utils::primitive_range(s.primitive_type(), m.top_simplex_type())) {
            SimplexCollection single_dim_comp(m, os_m.simplex_vector(pt));

            auto itrb = cofaces_single_dimension_iterable(m, s, pt);
            SimplexCollection itrb_collection(m);
            for (const Tuple& tt : itrb) {
                itrb_collection.add(simplex::Simplex(m, pt, tt));
            }
            REQUIRE(single_dim_comp.size() == itrb_collection.size());
            itrb_collection.sort_and_clean();
            REQUIRE(single_dim_comp.size() == itrb_collection.size());

            for (size_t i = 0; i < itrb_collection.simplex_vector().size(); ++i) {
                CHECK(simplex::utils::SimplexComparisons::equal(
                    m,
                    itrb_collection.simplex_vector()[i],
                    single_dim_comp.simplex_vector()[i]));
            }
        }
    };

    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        const simplex::Simplex s(m, PrimitiveType::Vertex, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        const simplex::Simplex s(m, PrimitiveType::Edge, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
        const simplex::Simplex s(m, PrimitiveType::Triangle, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Tetrahedron)) {
        const simplex::Simplex s(m, PrimitiveType::Tetrahedron, t);
        compare_collections(s);
    }
}

TEST_CASE("simplex_faces_single_dimension", "[simplex_collection]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    SECTION("vertices_0123")
    {
        const std::vector<int64_t> expected_vids{0, 1, 2, 3};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_with_vs_and_t(0, 1, 0));
        for (int64_t dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension_tuples(
                m,
                Simplex(m, get_primitive_type_from_id(dim), t),
                PrimitiveType::Vertex);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 2);
        CHECK(single_dim_faces[2].size() == 3);
        CHECK(single_dim_faces[3].size() == 4);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                CHECK(m.id(Simplex::vertex(m, vs[i])) == expected_vids[i]);
            }
        }
    }
    SECTION("vertices_1023")
    {
        const std::vector<int64_t> expected_vids{1, 0, 2, 3};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_with_vs_and_t(1, 0, 0));
        for (int64_t dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension_tuples(
                m,
                Simplex(m, get_primitive_type_from_id(dim), t),
                PrimitiveType::Vertex);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 2);
        CHECK(single_dim_faces[2].size() == 3);
        CHECK(single_dim_faces[3].size() == 4);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                CHECK(m.id(Simplex::vertex(m, vs[i])) == expected_vids[i]);
            }
        }
    }
    SECTION("edges_0123")
    {
        const std::vector<std::array<int64_t, 2>>
            expected_vids{{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_with_vs_and_t(0, 1, 0));
        for (int64_t dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension_tuples(
                m,
                Simplex(m, get_primitive_type_from_id(dim), t),
                PrimitiveType::Edge);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 0);
        CHECK(single_dim_faces[2].size() == 3);
        CHECK(single_dim_faces[3].size() == 6);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                const std::vector<Tuple> edge_vertices = faces_single_dimension_tuples(
                    m,
                    Simplex::edge(m, vs[i]),
                    PrimitiveType::Vertex);
                CHECK(edge_vertices.size() == 2);
                for (size_t j = 0; j < edge_vertices.size(); ++j) {
                    const int64_t ev = m.id(Simplex::vertex(m, edge_vertices[j]));
                    CHECK(ev == expected_vids[i][j]);
                }
            }
        }
    }
    SECTION("faces_0123")
    {
        const std::vector<std::array<int64_t, 3>> expected_vids{
            {0, 1, 2},
            {0, 3, 1},
            {1, 3, 2},
            {2, 3, 0}};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_with_vs_and_t(0, 1, 0));
        for (int64_t dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension_tuples(
                m,
                Simplex(m, get_primitive_type_from_id(dim), t),
                PrimitiveType::Triangle);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 0);
        CHECK(single_dim_faces[2].size() == 0);
        CHECK(single_dim_faces[3].size() == 4);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                const std::vector<Tuple> edge_vertices = faces_single_dimension_tuples(
                    m,
                    Simplex::face(m, vs[i]),
                    PrimitiveType::Vertex);
                CHECK(edge_vertices.size() == 3);
                for (size_t j = 0; j < edge_vertices.size(); ++j) {
                    const int64_t ev = m.id(Simplex::vertex(m, edge_vertices[j]));
                    CHECK(ev == expected_vids[i][j]);
                }
            }
        }
    }
}

TEST_CASE("simplex_compare_faces_with_faces_single_dimension", "[simplex_collection]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    for (int64_t cell_dim = 0; cell_dim < 4; ++cell_dim) {
        const PrimitiveType cell_type = get_primitive_type_from_id(cell_dim);

        const std::vector<Tuple> cells = m.get_all(cell_type);

        for (int64_t face_dim = 0; face_dim < 4; ++face_dim) {
            const PrimitiveType face_type = get_primitive_type_from_id(face_dim);

            for (const Tuple& cell : cells) {
                const Simplex cell_simplex = Simplex(m, cell_type, cell);
                const std::vector<Tuple> fsd_vec =
                    faces_single_dimension_tuples(m, cell_simplex, face_type);

                SimplexCollection face_collection(m);
                for (const Tuple& f : fsd_vec) {
                    face_collection.add(Simplex(m, face_type, f));
                }
                face_collection.sort_and_clean();
                const std::vector<Simplex> faces_single_dim =
                    face_collection.simplex_vector(face_type);

                SimplexCollection bndry = faces(m, cell_simplex);
                const std::vector<Simplex> bndry_single_dim = bndry.simplex_vector(face_type);

                REQUIRE(faces_single_dim.size() == bndry_single_dim.size());
                for (size_t i = 0; i < faces_single_dim.size(); ++i) {
                    CHECK(simplex::utils::SimplexComparisons::equal(
                        m,
                        bndry_single_dim[i],
                        faces_single_dim[i]));
                }
            }
        }
    }
}

TEST_CASE("simplex_link_condtion_edgemesh", "[simplex_collection]")
{
    SECTION("cases should succeed")
    {
        tests::DEBUG_EdgeMesh m0 = tests::two_segments();
        tests::DEBUG_EdgeMesh m1 = tests::loop_lines();
        tests::DEBUG_EdgeMesh m2 = tests::two_line_loop();

        Tuple t(0, -1, -1, 0);
        REQUIRE(link_condition(m0, t) == true);
        REQUIRE(link_condition(m1, t) == true);
        REQUIRE(link_condition(m2, t) == true);
    }

    SECTION("cases should fail")
    {
        tests::DEBUG_EdgeMesh m0 = tests::single_line();
        tests::DEBUG_EdgeMesh m1 = tests::self_loop();

        Tuple t(0, -1, -1, 0);
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
        Tuple t(0, 2, -1, 1);
        REQUIRE(link_condition(m, t) == false);
    }

    SECTION("case two neighbors")
    {
        tests::DEBUG_TriMesh m;
        m = tests::two_neighbors();
        Tuple t1 = m.edge_tuple_with_vs_and_t(0, 1, 0);
        Tuple t2 = m.edge_tuple_with_vs_and_t(1, 2, 0);
        Tuple t3 = m.edge_tuple_with_vs_and_t(2, 0, 2);
        REQUIRE(link_condition(m, t1) == false);
        REQUIRE(link_condition(m, t2) == true);
        REQUIRE(link_condition(m, t3) == false);
    }

    SECTION("case nine_triangles_with_a_hole")
    {
        tests::DEBUG_TriMesh m;
        m = tests::nine_triangles_with_a_hole();
        Tuple t1 = m.edge_tuple_with_vs_and_t(1, 2, 0);
        Tuple t2 = m.edge_tuple_with_vs_and_t(2, 4, 2);
        Tuple t3 = m.edge_tuple_with_vs_and_t(1, 6, 3);
        REQUIRE(link_condition(m, t1) == false);
        REQUIRE(link_condition(m, t2) == false);
        REQUIRE(link_condition(m, t3) == true);
    }
}

TEST_CASE("simplex_link_single_dimension_tri_iterable", "[simplex_collection][iterable][2D]")
{
    auto mp = std::make_unique<TriMesh>(tests::hex_plus_two());
    Mesh& m = *mp;

    auto compare_collections = [&m](const simplex::Simplex& s) {
        SimplexCollection comp = link_slow(m, s);
        for (const PrimitiveType pt :
             wmtk::utils::primitive_range(PrimitiveType::Vertex, m.top_simplex_type())) {
            SimplexCollection single_dim_comp(m, comp.simplex_vector(pt));

            auto itrb = link_single_dimension_iterable(m, s, pt);
            SimplexCollection itrb_collection(m);
            for (const Tuple& tt : itrb) {
                itrb_collection.add(simplex::Simplex(m, pt, tt));
            }
            REQUIRE(single_dim_comp.size() == itrb_collection.size());
            itrb_collection.sort_and_clean();
            REQUIRE(single_dim_comp.size() == itrb_collection.size());

            for (size_t i = 0; i < itrb_collection.simplex_vector().size(); ++i) {
                CHECK(simplex::utils::SimplexComparisons::equal(
                    m,
                    itrb_collection.simplex_vector()[i],
                    single_dim_comp.simplex_vector()[i]));
            }
        }
    };

    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        const simplex::Simplex s(m, PrimitiveType::Vertex, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        const simplex::Simplex s(m, PrimitiveType::Edge, t);
        compare_collections(s);
    }
}

TEST_CASE("simplex_link_single_dimension_tet_iterable", "[simplex_collection][iterable][3D]")
{
    auto mp = std::make_unique<TetMesh>(tests_3d::six_cycle_tets());
    Mesh& m = *mp;

    auto compare_collections = [&m](const simplex::Simplex& s) {
        SimplexCollection comp = link_slow(m, s);
        for (const PrimitiveType pt :
             wmtk::utils::primitive_range(PrimitiveType::Vertex, m.top_simplex_type())) {
            SimplexCollection single_dim_comp(m, comp.simplex_vector(pt));

            auto itrb = link_single_dimension_iterable(m, s, pt);
            SimplexCollection itrb_collection(m);
            for (const Tuple& tt : itrb) {
                itrb_collection.add(simplex::Simplex(m, pt, tt));
            }
            REQUIRE(single_dim_comp.size() == itrb_collection.size());
            itrb_collection.sort_and_clean();
            REQUIRE(single_dim_comp.size() == itrb_collection.size());

            for (size_t i = 0; i < itrb_collection.simplex_vector().size(); ++i) {
                CHECK(simplex::utils::SimplexComparisons::equal(
                    m,
                    itrb_collection.simplex_vector()[i],
                    single_dim_comp.simplex_vector()[i]));
            }
        }
    };

    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        const simplex::Simplex s(m, PrimitiveType::Vertex, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        const simplex::Simplex s(m, PrimitiveType::Edge, t);
        compare_collections(s);
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
        const simplex::Simplex s(m, PrimitiveType::Triangle, t);
        compare_collections(s);
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
            sc1.add(Simplex(m, PV, vertices[i]));
        }
        if (i < edges.size()) {
            sc1.add(Simplex(m, PE, edges[i]));
        }
        if (i < faces.size()) {
            sc1.add(Simplex(m, PF, faces[i]));
        }
    }

    for (int64_t i = 5; i >= 0; i--) {
        if (i % key == 1) continue;
        if (i < vertices.size()) {
            sc2.add(Simplex(m, PV, vertices[i]));
        }
        if (i < edges.size()) {
            sc2.add(Simplex(m, PE, edges[i]));
        }
        if (i < faces.size()) {
            sc2.add(Simplex(m, PF, faces[i]));
        }
    }
    sc1.sort_and_clean();
    sc2.sort_and_clean();
    REQUIRE(SimplexCollection::are_simplex_collections_equal(sc1, sc2) == true);
}

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
            const RawSimplex s0(m, Simplex::vertex(m, t));
            const RawSimplex s1(m, Simplex::vertex(m, m.switch_edge(t)));
            const RawSimplex s_edge(m, Simplex::edge(m, t));
            CHECK(s0 == s1);
            CHECK_FALSE(s0 == s_edge);
            CHECK(s0 < s_edge);
            if (m.is_boundary_edge(t)) {
                continue;
            }
            const RawSimplex s2(m, Simplex::vertex(m, m.switch_face(t)));
            CHECK(s0 == s2);
            CHECK(s1 == s2);
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PE);
        REQUIRE(edges.size() == 5);
        for (const Tuple& t : edges) {
            const RawSimplex s0(m, Simplex::edge(m, t));
            const RawSimplex s1(m, Simplex::edge(m, m.switch_vertex(t)));
            CHECK_FALSE(s0 < s1);
            CHECK_FALSE(s1 < s0);
            CHECK(s0 == s1);
            if (m.is_boundary_edge(t)) {
                continue;
            }
            const RawSimplex s2(m, Simplex::edge(m, m.switch_face(t)));
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
            const RawSimplex s0(m, Simplex::face(m, t));
            const RawSimplex s1(m, Simplex::face(m, m.switch_vertex(t)));
            CHECK_FALSE(s0 < s1);
            CHECK_FALSE(s1 < s0);
            CHECK(s0 == s1);
            const RawSimplex s2(m, Simplex::face(m, m.switch_edge(t)));
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
        raw_simplex_collection.add(m, Simplex::edge(m, t));
    }
    for (const Tuple& t : faces) {
        raw_simplex_collection.add(m, Simplex::face(m, t));
    }
    CHECK(raw_simplex_collection.simplex_vector().size() == 11);

    // test sorting and clean-up
    for (const Tuple& t : vertices) {
        raw_simplex_collection.add(m, Simplex::vertex(m, t));
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

    for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
        sc.add(m, Simplex::face(m, t));
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        if (!m.is_boundary_edge(t)) {
            continue;
        }
        std::vector<Tuple> vertices =
            faces_single_dimension_tuples(m, Simplex::edge(m, t), PrimitiveType::Vertex);
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
        const Tuple t = m.edge_tuple_with_vs_and_t(13, 14, 17);

        SimplexCollection sc = link(m, Simplex::vertex(m, t));

        REQUIRE(sc.simplex_vector().size() == 98);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 32);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 48);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 18);

        const auto& simplices = sc.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(m, t);
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

        for (size_t i = 18; i < 18 + 48; ++i) {
            const Simplex& e = simplices[i];
            int id0 = m.id(e.tuple(), PrimitiveType::Vertex);
            int id1 = m.id(m.switch_vertex(e.tuple()), PrimitiveType::Vertex);
            m.face_tuple_from_vids(id0, id1, 13);
        }

        int sum = 0;
        for (size_t i = 18 + 48; i < 98; ++i) {
            const Simplex& f = simplices[i];
            int id0 = m.id(f.tuple(), PrimitiveType::Vertex);
            int id1 = m.id(m.switch_vertex(f.tuple()), PrimitiveType::Vertex);
            int id2 = m.id(m.switch_vertex(m.switch_edge(f.tuple())), PrimitiveType::Vertex);
            m.tet_tuple_from_vids(id0, id1, id2, 13);
        }
    }
    SECTION("vertex_boundary")
    {
        const Tuple t = m.edge_tuple_with_vs_and_t(14, 13, 8);

        CHECK(m.id(t, PrimitiveType::Vertex) == 14);

        SimplexCollection sc = link(m, Simplex::vertex(m, t));

        REQUIRE(sc.simplex_vector().size() == 17);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 4);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 8);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 5);

        const auto& simplices = sc.simplex_vector();
        const simplex::Simplex v = simplex::Simplex::vertex(m, t);
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
        const Tuple t = m.edge_tuple_with_vs_and_t(9, 13, 2);

        SimplexCollection sc = link(m, Simplex::edge(m, t));

        REQUIRE(sc.simplex_vector().size() == 12);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 0);
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
        const Tuple t = m.edge_tuple_with_vs_and_t(1, 3, 0);

        SimplexCollection sc = link(m, Simplex::edge(m, t));

        REQUIRE(sc.simplex_vector().size() == 7);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 0);
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

        SimplexCollection sc = link(m, Simplex::face(m, t));

        REQUIRE(sc.simplex_vector().size() == 2);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 2);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 1);
        CHECK(m.id(simplices[1]) == 19);
    }
    SECTION("face_boundary")
    {
        const Tuple t = m.face_tuple_from_vids(0, 1, 3);

        SimplexCollection sc = link(m, Simplex::face(m, t));

        REQUIRE(sc.simplex_vector().size() == 1);
        CHECK(sc.simplex_vector(PrimitiveType::Triangle).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(sc.simplex_vector(PrimitiveType::Vertex).size() == 1);

        const auto& simplices = sc.simplex_vector();

        CHECK(m.id(simplices[0]) == 9);
    }
}
