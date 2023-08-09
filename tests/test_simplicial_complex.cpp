// test code for SC
// Psudo code now

// #include "SimplicialComplex.hpp"
#include <catch2/catch_test_macros.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
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
            const Simplex s0(PV, t);
            const Simplex s1(PV, m.switch_tuple(t, PE));
            CHECK(m.simplex_is_equal(s0, s1));
            if (m.is_boundary(t)) {
                continue;
            }
            const Simplex s2(PV, m.switch_tuple(t, PF));
            CHECK(m.simplex_is_equal(s0, s2));
            CHECK(m.simplex_is_equal(s1, s2));
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PE);
        REQUIRE(edges.size() == 5);
        for (const Tuple& t : edges) {
            const Simplex s0(PE, t);
            const Simplex s1(PE, m.switch_tuple(t, PV));
            CHECK_FALSE(m.simplex_is_less(s0, s1));
            CHECK_FALSE(m.simplex_is_less(s1, s0));
            if (m.is_boundary(t)) {
                continue;
            }
            const Simplex s2(PE, m.switch_tuple(t, PF));
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
            const Simplex s0(PF, t);
            const Simplex s1(PF, m.switch_tuple(t, PV));
            CHECK_FALSE(m.simplex_is_less(s0, s1));
            CHECK_FALSE(m.simplex_is_less(s1, s0));
            const Simplex s2(PF, m.switch_tuple(t, PE));
            CHECK_FALSE(m.simplex_is_less(s0, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s0));
            CHECK_FALSE(m.simplex_is_less(s1, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s1));
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
    internal::SimplexSet simplices(slf);
    for (const auto& t : vertices) {
        simplices.insert(Simplex(PV, t));
    }
    for (const auto& t : edges) {
        simplices.insert(Simplex(PE, t));
    }
    for (const auto& t : faces) {
        simplices.insert(Simplex(PF, t));
    }
    // try inserting a valence 1 vertex
    REQUIRE(simplices.size() == 11);
    auto [it, was_successful] = simplices.insert(Simplex(PV, vertices[2]));
    REQUIRE_FALSE(was_successful);
    REQUIRE(simplices.size() == 11);
    std::tie(it, was_successful) = simplices.insert(Simplex(PV, m.switch_tuple(vertices[2], PE)));
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
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);

    // make sure this is the right tuple
    REQUIRE(m.id(t, PF) == 1);
    REQUIRE(m.id(t, PV) == 0);
    REQUIRE(m.id(m.switch_tuple(t, PV), PV) == 1);
    REQUIRE(m.id(m.switch_tuple(m.switch_tuple(t, PE), PV), PV) == 3);


    SimplicialComplex lnk_0 = SimplicialComplex::link(Simplex(PV, t), m);
    SimplicialComplex lnk_1 = SimplicialComplex::link(Simplex(PV, m.switch_tuple(t, PV)), m);


    SimplicialComplex lhs = SimplicialComplex::get_intersection(lnk_0, lnk_1);
    SimplicialComplex lnk_01 = SimplicialComplex::link(Simplex(PE, t), m);

    SimplicialComplex lnk_10 = SimplicialComplex::link(Simplex(PE, m.switch_tuple(t, PV)), m);

    REQUIRE(lnk_0.get_simplices().size() == 5);
    REQUIRE(lnk_1.get_simplices().size() == 5);

    REQUIRE(lnk_01.get_simplices().size() == 1);
    REQUIRE(lhs.get_simplices().size() == 3);
    REQUIRE(lnk_01 == lnk_10);

    REQUIRE(SimplicialComplex::link_cond(m, t) == false);
    REQUIRE(SimplicialComplex::link_cond_bd_2d(m, t) == false);
    REQUIRE(SimplicialComplex::edge_collapse_possible_2d(m, t) == true);
}


TEST_CASE("link-case2", "[simplicial_complex][link][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::three_neighbors();

    // get the tuple point to V(0), E(01), F(012)
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);
    // make sure this is the right tuple
    REQUIRE(m.id(t, PF) == 1);
    REQUIRE(m.id(t, PV) == 0);
    REQUIRE(m.id(m.switch_tuple(t, PV), PV) == 1);
    REQUIRE(m.id(m.switch_tuple(m.switch_tuple(t, PE), PV), PV) == 2);

    SimplicialComplex lnk_0 = SimplicialComplex::link(Simplex(PV, t), m);
    SimplicialComplex lnk_1 = SimplicialComplex::link(Simplex(PV, m.switch_tuple(t, PV)), m);


    SimplicialComplex lhs = SimplicialComplex::get_intersection(lnk_0, lnk_1);
    SimplicialComplex lnk_01 = SimplicialComplex::link(Simplex(PE, t), m);
    SimplicialComplex lnk_10 = SimplicialComplex::link(Simplex(PE, m.switch_tuple(t, PV)), m);


    REQUIRE(lnk_0.get_simplices().size() == 7);
    REQUIRE(lnk_1.get_simplices().size() == 7);
    REQUIRE(lnk_01.get_simplices().size() == 2);

    REQUIRE(lhs == lnk_01);
    REQUIRE(lnk_01 == lnk_10);

    REQUIRE(SimplicialComplex::link_cond(m, t) == true);
    REQUIRE(SimplicialComplex::link_cond_bd_2d(m, t) == false);
    REQUIRE(SimplicialComplex::edge_collapse_possible_2d(m, t) == false);
}

TEST_CASE("k-ring", "[simplicial_complex][k-ring][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::three_neighbors();

    // get the tuple point to V(3)
    long hash = 0;
    Tuple t(1, 0, -1, 0, hash);
    REQUIRE(m.id(t, PV) == 3);

    auto ret1 = SimplicialComplex::vertex_one_ring(m, t);
    REQUIRE(ret1.size() == 2);
    auto ret2 = SimplicialComplex::k_ring(m, t, 1);
    REQUIRE(ret2.size() == 2);
    auto ret3 = SimplicialComplex::k_ring(m, t, 2);
    REQUIRE(ret3.size() == 6);
    auto ret4 = SimplicialComplex::k_ring(m, t, 3);
    REQUIRE(ret4.size() == 6);
}

TEST_CASE("open_star", "[simplicial_complex][star][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::three_neighbors();

    // get the tuple point to V(0), E(01), F(012)
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);


    SimplicialComplex sc_v = SimplicialComplex::open_star(Simplex(PV, t), m);
    REQUIRE(sc_v.get_simplices().size() == 8);

    SimplicialComplex sc_e = SimplicialComplex::open_star(Simplex(PE, t), m);
    REQUIRE(sc_e.get_simplices().size() == 3);

    SimplicialComplex sc_f = SimplicialComplex::open_star(Simplex(PF, t), m);
    REQUIRE(sc_f.get_simplices().size() == 1);
}

TEST_CASE("closed_star", "[simplicial_complex][star][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::three_neighbors();

    // get the tuple point to V(0), E(01), F(012)
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);
    REQUIRE(m.id(t, PV) == 0);
    REQUIRE(m.id(m.switch_tuple(t, PV), PV) == 1);
    REQUIRE(m.id(m.switch_tuple(m.switch_tuple(t, PE), PV), PV) == 2);


    SimplicialComplex sc_v = SimplicialComplex::closed_star(Simplex(PV, t), m);
    REQUIRE(sc_v.get_simplices().size() == 15);

    SimplicialComplex sc_e = SimplicialComplex::closed_star(Simplex(PE, t), m);
    REQUIRE(sc_e.get_simplices().size() == 11);

    SimplicialComplex sc_f = SimplicialComplex::closed_star(Simplex(PF, t), m);
    REQUIRE(sc_f.get_simplices().size() == 7);
}
