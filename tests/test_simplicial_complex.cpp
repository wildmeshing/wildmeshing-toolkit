// test code for SC
// Psudo code now
#include <catch2/catch_test_macros.hpp>
#include <set>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/internal/SimplexLessFunctor.hpp>
#include <wmtk/simplex/k_ring.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"


using namespace wmtk;
using namespace wmtk::simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;


TEST_CASE("simplex_comparison", "[simplicial_complex][2D]")
{
    // switching anything in a tuple besides the currently viewed simplex must not change the
    // simplex

    TriMesh m = tests::quad();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PV);
        REQUIRE(vertices.size() == 4);
        for (const Tuple& t : vertices) {
            const Simplex s0(m, PV, t);
            const Simplex s1(m, PV, m.switch_tuple(t, PE));
            CHECK(simplex::utils::SimplexComparisons::equal(m, s0, s1));
            if (m.is_boundary_edge(t)) {
                continue;
            }
            const Simplex s2(m, PV, m.switch_tuple(t, PF));
            CHECK(simplex::utils::SimplexComparisons::equal(m, s0, s2));
            CHECK(simplex::utils::SimplexComparisons::equal(m, s1, s2));
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PE);
        REQUIRE(edges.size() == 5);
        for (const Tuple& t : edges) {
            const Simplex s0(m, PE, t);
            const Simplex s1(m, PE, m.switch_tuple(t, PV));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s0, s1));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s1, s0));
            if (m.is_boundary_edge(t)) {
                continue;
            }
            const Simplex s2(m, PE, m.switch_tuple(t, PF));
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
            const Simplex s0(m, PF, t);
            const Simplex s1(m, PF, m.switch_tuple(t, PV));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s0, s1));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s1, s0));
            const Simplex s2(m, PF, m.switch_tuple(t, PE));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s0, s2));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s2, s0));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s1, s2));
            CHECK_FALSE(simplex::utils::SimplexComparisons::less(m, s2, s1));
        }
    }
}

TEST_CASE("simplex_set", "[simplicial_complex][2D]")
{
    TriMesh m = tests::quad();
    const std::vector<Tuple> vertices = m.get_all(PV);
    REQUIRE(vertices.size() == 4);
    const std::vector<Tuple> edges = m.get_all(PE);
    REQUIRE(edges.size() == 5);
    const std::vector<Tuple> faces = m.get_all(PF);
    REQUIRE(faces.size() == 2);

    const internal::SimplexLessFunctor slf(m);
    std::set<Simplex, internal::SimplexLessFunctor> simplices(slf);
    for (const auto& t : vertices) {
        simplices.insert(Simplex(m, PV, t));
    }
    for (const auto& t : edges) {
        simplices.insert(Simplex(m, PE, t));
    }
    for (const auto& t : faces) {
        simplices.insert(Simplex(m, PF, t));
    }
    // try inserting a valence 1 vertex
    REQUIRE(simplices.size() == 11);
    auto [it, was_successful] = simplices.insert(Simplex(m, PV, vertices[2]));
    REQUIRE_FALSE(was_successful);
    REQUIRE(simplices.size() == 11);
    std::tie(it, was_successful) =
        simplices.insert(Simplex(m, PV, m.switch_tuple(vertices[2], PE)));
    REQUIRE_FALSE(was_successful);
    REQUIRE(simplices.size() == 11);
}

TEST_CASE("link-case1", "[simplicial_complex][link][2D]")
{
    RowVectors3l F(3, 3);
    F << 0, 3, 2, 0, 1, 3, 1, 2, 3; // 3 Faces
    //  triangular cone with 3 as the top / tetrahedron with 0 1 2 missing
    //  2------ 3 ---- 2
    //   |     / \     |
    //   |    /   \    |
    //   |   /  ^  \   |
    //   |  /       \  |
    //   0  ---------  1
    //   ^      ^
    //   Tuple is identified by the above arrows

    // dump it to (Tri)Mesh
    tests::DEBUG_TriMesh m;
    m.initialize(F);

    // get the tuple point to V(0), E(01), F(013)
    Tuple t(0, 2, -1, 1);

    // make sure this is the right tuple
    REQUIRE(m.id(t, PF) == 1);
    REQUIRE(m.id(t, PV) == 0);
    REQUIRE(m.id(m.switch_tuple(t, PV), PV) == 1);
    REQUIRE(m.id(m.switch_tuple(m.switch_tuple(t, PE), PV), PV) == 3);


    SimplexCollection lnk_0 = link(m, Simplex(m, PV, t));
    SimplexCollection lnk_1 = link(m, Simplex(m, PV, m.switch_tuple(t, PV)));


    SimplexCollection lhs = SimplexCollection::get_intersection(lnk_0, lnk_1);
    SimplexCollection lnk_01 = link(m, Simplex(m, PE, t));

    SimplexCollection lnk_10 = link(m, Simplex(m, PE, m.switch_tuple(t, PV)));

    REQUIRE(lnk_0.simplex_vector().size() == 5);
    REQUIRE(lnk_1.simplex_vector().size() == 5);

    REQUIRE(lnk_01.simplex_vector().size() == 1);
    REQUIRE(lhs.simplex_vector().size() == 3);
    REQUIRE(lnk_01 == lnk_10);

    // Old code
    //  REQUIRE(link_cond(m, t) == false);
    //  REQUIRE(link_cond_bd_2d(m, t) == false);
    //  REQUIRE(edge_collapse_possible_2d(m, t) == true);
}


TEST_CASE("link-case2", "[simplicial_complex][link][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::three_neighbors();

    // get the tuple point to V(0), E(01), F(012)
    Tuple t(0, 2, -1, 1);
    // make sure this is the right tuple
    REQUIRE(m.id(t, PF) == 1);
    REQUIRE(m.id(t, PV) == 0);
    REQUIRE(m.id(m.switch_tuple(t, PV), PV) == 1);
    REQUIRE(m.id(m.switch_tuple(m.switch_tuple(t, PE), PV), PV) == 2);

    SimplexCollection lnk_0 = link(m, Simplex(m, PV, t));
    SimplexCollection lnk_1 = link(m, Simplex(m, PV, m.switch_tuple(t, PV)));


    SimplexCollection lhs = SimplexCollection::get_intersection(lnk_0, lnk_1);
    SimplexCollection lnk_01 = link(m, Simplex(m, PE, t));
    SimplexCollection lnk_10 = link(m, Simplex(m, PE, m.switch_tuple(t, PV)));


    REQUIRE(lnk_0.simplex_vector().size() == 7);
    REQUIRE(lnk_1.simplex_vector().size() == 7);
    REQUIRE(lnk_01.simplex_vector().size() == 2);

    REQUIRE(lhs == lnk_01);
    REQUIRE(lnk_01 == lnk_10);

    // Old code
    // REQUIRE(link_cond(m, t) == true);
    // REQUIRE(link_cond_bd_2d(m, t) == false);
    // REQUIRE(edge_collapse_possible_2d(m, t) == false);
}


// Old code
// TEST_CASE("link-condition-edgemesh", "[simplicial_complex][link][1D]")
// {
//     SECTION("cases should succeed")
//     {
//         tests::DEBUG_EdgeMesh m0 = tests::two_segments();
//         tests::DEBUG_EdgeMesh m1 = tests::loop_lines();
//         tests::DEBUG_EdgeMesh m2 = tests::two_line_loop();

//         int64_t hash = 0;
//         Tuple t(0, -1, -1, 0, hash);
//         REQUIRE(link_cond_bd_1d(m0, t) == true);
//         REQUIRE(link_cond_bd_1d(m1, t) == true);
//         REQUIRE(link_cond_bd_1d(m2, t) == true);
//     }

//     SECTION("cases should fail")
//     {
//         tests::DEBUG_EdgeMesh m0 = tests::single_line();
//         tests::DEBUG_EdgeMesh m1 = tests::self_loop();

//         int64_t hash = 0;
//         Tuple t(0, -1, -1, 0, hash);
//         REQUIRE(link_cond_bd_1d(m0, t) == false);
//         REQUIRE(link_cond_bd_1d(m1, t) == false);
//     }
// }

TEST_CASE("k-ring", "[simplicial_complex][k-ring][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::three_neighbors();

    // get the tuple point to V(3)
    Tuple t(1, 0, -1, 0);
    Simplex s = Simplex::vertex(m, t);
    REQUIRE(m.id(t, PV) == 3);
    // Old code
    //  const auto ret0 = vertex_one_ring(static_cast<Mesh&>(m), t);
    //  CHECK(ret0.size() == 2);
    //  const auto ret1 = vertex_one_ring(m, t);
    //  CHECK(ret1.size() == 2);
    const auto ret2 = k_ring(m, s, 1).simplex_vector(PrimitiveType::Vertex);
    CHECK(ret2.size() == 2);
    const auto ret3 = k_ring(m, s, 2).simplex_vector(PrimitiveType::Vertex);
    CHECK(ret3.size() == 6);
    const auto ret4 = k_ring(m, s, 3).simplex_vector(PrimitiveType::Vertex);
    CHECK(ret4.size() == 6);
}

TEST_CASE("vertex_one_ring", "[simplicial_complex][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::hex_plus_two();

    Tuple t;
    std::vector<Simplex> ring0;
    SECTION("interior")
    {
        t = m.edge_tuple_with_vs_and_t(4, 5, 2);
        auto sc = link(static_cast<Mesh&>(m), Simplex::vertex(m, t));
        ring0 = sc.simplex_vector(PrimitiveType::Vertex);
        CHECK(ring0.size() == 6);
        CHECK(sc.size() == 12);
    }
    SECTION("on_boundary_cw")
    {
        t = m.edge_tuple_with_vs_and_t(0, 1, 1);
        auto sc = link(static_cast<Mesh&>(m), Simplex::vertex(m, t));
        ring0 = sc.simplex_vector(PrimitiveType::Vertex);
        CHECK(ring0.size() == 3);
        CHECK(sc.size() == 5);
    }
    SECTION("on_boundary_ccw")
    {
        t = m.edge_tuple_with_vs_and_t(0, 3, 0);
        auto sc = link(static_cast<Mesh&>(m), Simplex::vertex(m, t));
        ring0 = sc.simplex_vector(PrimitiveType::Vertex);
        CHECK(ring0.size() == 3);
        CHECK(sc.size() == 5);
    }
    SECTION("single_boundary_triangle_cw")
    {
        t = m.edge_tuple_with_vs_and_t(6, 5, 4);
        ring0 = link(static_cast<Mesh&>(m), Simplex::vertex(m, t))
                    .simplex_vector(PrimitiveType::Vertex);
        CHECK(ring0.size() == 2);
    }
    SECTION("single_boundary_triangle_ccw")
    {
        t = m.edge_tuple_with_vs_and_t(6, 2, 4);
        ring0 = link(static_cast<Mesh&>(m), Simplex::vertex(m, t))
                    .simplex_vector(PrimitiveType::Vertex);
        CHECK(ring0.size() == 2);
    }

    const auto ret1 = link(m, Simplex::vertex(m, t)).simplex_vector(PrimitiveType::Vertex);
    CHECK(ring0.size() == ret1.size());
}

TEST_CASE("open_star", "[simplicial_complex][star][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::three_neighbors();

    // get the tuple point to V(0), E(01), F(012)
    const Tuple t(0, 2, -1, 1);
    const simplex::IdSimplex v = m.get_id_simplex(t, PV);
    const simplex::IdSimplex e = m.get_id_simplex(t, PE);
    const simplex::IdSimplex f = m.get_id_simplex(t, PF);


    auto sc_v = open_star(m, Simplex(m, PV, t)).simplex_vector();
    REQUIRE(sc_v.size() == 8);
    for (size_t i = 0; i < 8; i++) {
        REQUIRE(v == sc_v[i]);
    }

    auto sc_e = open_star(m, Simplex(m, PE, t)).simplex_vector();
    REQUIRE(sc_e.size() == 3);
    for (size_t i = 0; i < 3; i++) {
        REQUIRE(e == sc_e[i]);
    }

    auto sc_f = open_star(m, Simplex(m, PF, t)).simplex_vector();
    REQUIRE(sc_f.size() == 1);
    for (size_t i = 0; i < 1; i++) {
        REQUIRE(f == sc_f[i]);
    }
}

TEST_CASE("closed_star", "[simplicial_complex][star][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::three_neighbors();

    // get the tuple point to V(0), E(01), F(012)
    Tuple t(0, 2, -1, 1);
    REQUIRE(m.id(t, PV) == 0);
    REQUIRE(m.id(m.switch_tuple(t, PV), PV) == 1);
    REQUIRE(m.id(m.switch_tuple(m.switch_tuple(t, PE), PV), PV) == 2);


    SimplexCollection sc_v = closed_star(m, Simplex(m, PV, t));
    REQUIRE(sc_v.simplex_vector().size() == 15);

    SimplexCollection sc_e = closed_star(m, Simplex(m, PE, t));
    REQUIRE(sc_e.simplex_vector().size() == 11);

    SimplexCollection sc_f = closed_star(m, Simplex(m, PF, t));
    REQUIRE(sc_f.simplex_vector().size() == 7);
}


TEST_CASE("open_star_3d", "[simplicial_complex][open_star][3D]")
{
    tests_3d::DEBUG_TetMesh m;
    m = tests_3d::single_tet();

    const Tuple t = m.edge_tuple_with_vs_and_t(1, 2, 0);

    SimplexCollection sc_v = open_star(m, Simplex(m, PV, t));
    CHECK(sc_v.simplex_vector(PV).size() == 1);
    CHECK(sc_v.simplex_vector(PE).size() == 3);
    CHECK(sc_v.simplex_vector(PF).size() == 3);
    CHECK(sc_v.simplex_vector(PT).size() == 1);

    SimplexCollection sc_e = open_star(m, Simplex(m, PE, t));
    CHECK(sc_e.simplex_vector(PV).size() == 0);
    CHECK(sc_e.simplex_vector(PE).size() == 1);
    CHECK(sc_e.simplex_vector(PF).size() == 2);
    CHECK(sc_e.simplex_vector(PT).size() == 1);

    SimplexCollection sc_f = open_star(m, Simplex(m, PF, t));
    CHECK(sc_f.simplex_vector(PV).size() == 0);
    CHECK(sc_f.simplex_vector(PE).size() == 0);
    CHECK(sc_f.simplex_vector(PF).size() == 1);
    CHECK(sc_f.simplex_vector(PT).size() == 1);

    SimplexCollection sc_t = open_star(m, Simplex(m, PT, t));
    CHECK(sc_t.simplex_vector(PV).size() == 0);
    CHECK(sc_t.simplex_vector(PE).size() == 0);
    CHECK(sc_t.simplex_vector(PF).size() == 0);
    CHECK(sc_t.simplex_vector(PT).size() == 1);
}
