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


    SimplicialComplex lnk_0 = SimplicialComplex::link(m, Simplex(PV, t));
    SimplicialComplex lnk_1 = SimplicialComplex::link(m, Simplex(PV, m.switch_tuple(t, PV)));


    SimplicialComplex lhs = SimplicialComplex::get_intersection(lnk_0, lnk_1);
    SimplicialComplex lnk_01 = SimplicialComplex::link(m, Simplex(PE, t));

    SimplicialComplex lnk_10 = SimplicialComplex::link(m, Simplex(PE, m.switch_tuple(t, PV)));

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

    SimplicialComplex lnk_0 = SimplicialComplex::link(m, Simplex(PV, t));
    SimplicialComplex lnk_1 = SimplicialComplex::link(m, Simplex(PV, m.switch_tuple(t, PV)));


    SimplicialComplex lhs = SimplicialComplex::get_intersection(lnk_0, lnk_1);
    SimplicialComplex lnk_01 = SimplicialComplex::link(m, Simplex(PE, t));
    SimplicialComplex lnk_10 = SimplicialComplex::link(m, Simplex(PE, m.switch_tuple(t, PV)));


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

    const auto ret0 = SimplicialComplex::vertex_one_ring(static_cast<Mesh&>(m), t);
    CHECK(ret0.size() == 2);
    const auto ret1 = SimplicialComplex::vertex_one_ring(m, t);
    CHECK(ret1.size() == 2);
    const auto ret2 = SimplicialComplex::k_ring(m, t, 1);
    CHECK(ret2.size() == 2);
    const auto ret3 = SimplicialComplex::k_ring(m, t, 2);
    CHECK(ret3.size() == 6);
    const auto ret4 = SimplicialComplex::k_ring(m, t, 3);
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
        t = m.edge_tuple_between_v1_v2(4, 5, 2);
        ring0 = SimplicialComplex::vertex_one_ring(static_cast<Mesh&>(m), t);
        CHECK(ring0.size() == 6);
    }
    SECTION("on_boundary_cw")
    {
        t = m.edge_tuple_between_v1_v2(0, 1, 1);
        ring0 = SimplicialComplex::vertex_one_ring(static_cast<Mesh&>(m), t);
        CHECK(ring0.size() == 3);
    }
    SECTION("on_boundary_ccw")
    {
        t = m.edge_tuple_between_v1_v2(0, 3, 0);
        ring0 = SimplicialComplex::vertex_one_ring(static_cast<Mesh&>(m), t);
        CHECK(ring0.size() == 3);
    }
    SECTION("single_boundary_triangle_cw")
    {
        t = m.edge_tuple_between_v1_v2(6, 5, 4);
        ring0 = SimplicialComplex::vertex_one_ring(static_cast<Mesh&>(m), t);
        CHECK(ring0.size() == 2);
    }
    SECTION("single_boundary_triangle_ccw")
    {
        t = m.edge_tuple_between_v1_v2(6, 2, 4);
        ring0 = SimplicialComplex::vertex_one_ring(static_cast<Mesh&>(m), t);
        CHECK(ring0.size() == 2);
    }

    const auto ret1 = SimplicialComplex::vertex_one_ring(m, t);
    CHECK(ring0.size() == ret1.size());
}

TEST_CASE("open_star", "[simplicial_complex][star][2D]")
{
    tests::DEBUG_TriMesh m;
    m = tests::three_neighbors();

    // get the tuple point to V(0), E(01), F(012)
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);


    auto sc_v = SimplicialComplex::open_star(m, Simplex(PV, t)).get_simplex_vector();
    REQUIRE(sc_v.size() == 8);
    for (size_t i = 0; i < 8; i++)
    {
        REQUIRE(m.simplex_is_equal(Simplex(PV, t), Simplex(PV, sc_v[i].tuple())));
    }

    auto sc_e = SimplicialComplex::open_star(m, Simplex(PE, t)).get_simplex_vector();
    REQUIRE(sc_e.size() == 3);
    for (size_t i = 0; i < 3; i++)
    {
        REQUIRE(m.simplex_is_equal(Simplex(PE, t), Simplex(PE, sc_e[i].tuple())));
    }

    auto sc_f = SimplicialComplex::open_star(m, Simplex(PF, t)).get_simplex_vector();
    REQUIRE(sc_f.size() == 1);
    for (size_t i = 0; i < 1; i++)
    {
        REQUIRE(m.simplex_is_equal(Simplex(PF, t), Simplex(PF, sc_f[i].tuple())));
    }
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


    SimplicialComplex sc_v = SimplicialComplex::closed_star(m, Simplex(PV, t));
    REQUIRE(sc_v.get_simplices().size() == 15);

    SimplicialComplex sc_e = SimplicialComplex::closed_star(m, Simplex(PE, t));
    REQUIRE(sc_e.get_simplices().size() == 11);

    SimplicialComplex sc_f = SimplicialComplex::closed_star(m, Simplex(PF, t));
    REQUIRE(sc_f.get_simplices().size() == 7);
}


std::vector<std::vector<long>> get_sorted_sc(const tests::DEBUG_TriMesh &m, const std::vector<Simplex> &sc)
{
    std::vector<std::vector<long>> ret;
    for (auto s : sc)
    {
        std::vector<long> s_vec;
        Tuple t = s.tuple();
        switch (s.primitive_type())
        {
        case PV:
            s_vec.push_back(m.id(t, PV));
            break;
        case PE:
            s_vec.push_back(m.id(t, PV));
            s_vec.push_back(m.id(m.switch_vertex(t), PV));
            break;
        case PF:
            s_vec.push_back(m.id(t, PV));
            s_vec.push_back(m.id(m.switch_vertex(t), PV));
            s_vec.push_back(m.id(m.switch_vertex(m.switch_edge(t)), PV));
            /* code */
            break;
        case PT:
            // TODO: need implement for tet
            /* code */
            break;
        default:
            break;
        }
        std::sort(s_vec.begin(), s_vec.end());
        ret.push_back(s_vec);
    }
    std::sort(ret.begin(), ret.end(), [](const std::vector<long>& a, const std::vector<long>& b) {
        if (a.size() != b.size()) {
            return a.size() < b.size();
        }
        return a < b;
    });
    return ret;
}

#include <igl/read_triangle_mesh.h>
TEST_CASE("open_star_circle", "[simplicial_complex][open_star][2D]")
{
    RowVectors3d V;
    RowVectors3l F;
    std::string name = "/circle.obj";
    std::string path;
    path.append(WMTK_DATA_DIR);
    path.append(name);
    igl::read_triangle_mesh(path, V, F);
    tests::DEBUG_TriMesh m;
    m.initialize(F);

    Tuple t;
    std::vector<std::vector<long>> sc_v, sc_e, sc_f;
    t = m.tuple_from_id(PV, 3802);
    sc_v = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 13);
    CHECK(sc_v[0][0] == 3802);
    CHECK(sc_v[1][0] == 200);
    CHECK(sc_v[1][1] == 3802);
    CHECK(sc_v[2][0] == 201);
    CHECK(sc_v[2][1] == 3802);
    CHECK(sc_v[3][0] == 3772);
    CHECK(sc_v[3][1] == 3802);
    CHECK(sc_v[4][0] == 3773);
    CHECK(sc_v[4][1] == 3802);
    CHECK(sc_v[5][0] == 3801);
    CHECK(sc_v[5][1] == 3802);
    CHECK(sc_v[6][0] == 3802);
    CHECK(sc_v[6][1] == 3803);
    CHECK(sc_v[7][0] == 200);
    CHECK(sc_v[7][1] == 201);
    CHECK(sc_v[7][2] == 3802);
    CHECK(sc_v[8][0] == 200);
    CHECK(sc_v[8][1] == 3802);
    CHECK(sc_v[8][2] == 3803);
    CHECK(sc_v[9][0] == 201);
    CHECK(sc_v[9][1] == 3801);
    CHECK(sc_v[9][2] == 3802);
    CHECK(sc_v[10][0] == 3772);
    CHECK(sc_v[10][1] == 3773);
    CHECK(sc_v[10][2] == 3802);
    CHECK(sc_v[11][0] == 3772);
    CHECK(sc_v[11][1] == 3801);
    CHECK(sc_v[11][2] == 3802);
    CHECK(sc_v[12][0] == 3773);
    CHECK(sc_v[12][1] == 3802);
    CHECK(sc_v[12][2] == 3803);

    t = m.tuple_from_id(PV, 2587);
    sc_v = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 13);
    CHECK(sc_v[0][0] == 2587);
    CHECK(sc_v[1][0] == 2575);
    CHECK(sc_v[1][1] == 2587);
    CHECK(sc_v[2][0] == 2576);
    CHECK(sc_v[2][1] == 2587);
    CHECK(sc_v[3][0] == 2586);
    CHECK(sc_v[3][1] == 2587);
    CHECK(sc_v[4][0] == 2587);
    CHECK(sc_v[4][1] == 2588);
    CHECK(sc_v[5][0] == 2587);
    CHECK(sc_v[5][1] == 2599);
    CHECK(sc_v[6][0] == 2587);
    CHECK(sc_v[6][1] == 2600);
    CHECK(sc_v[7][0] == 2575);
    CHECK(sc_v[7][1] == 2576);
    CHECK(sc_v[7][2] == 2587);
    CHECK(sc_v[8][0] == 2575);
    CHECK(sc_v[8][1] == 2586);
    CHECK(sc_v[8][2] == 2587);
    CHECK(sc_v[9][0] == 2576);
    CHECK(sc_v[9][1] == 2587);
    CHECK(sc_v[9][2] == 2588);
    CHECK(sc_v[10][0] == 2586);
    CHECK(sc_v[10][1] == 2587);
    CHECK(sc_v[10][2] == 2599);
    CHECK(sc_v[11][0] == 2587);
    CHECK(sc_v[11][1] == 2588);
    CHECK(sc_v[11][2] == 2600);
    CHECK(sc_v[12][0] == 2587);
    CHECK(sc_v[12][1] == 2599);
    CHECK(sc_v[12][2] == 2600);

    t = m.tuple_from_id(PV, 1050);
    sc_v = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 13);
    CHECK(sc_v[0][0] == 1050);
    CHECK(sc_v[1][0] == 1036);
    CHECK(sc_v[1][1] == 1050);
    CHECK(sc_v[2][0] == 1037);
    CHECK(sc_v[2][1] == 1050);
    CHECK(sc_v[3][0] == 1049);
    CHECK(sc_v[3][1] == 1050);
    CHECK(sc_v[4][0] == 1050);
    CHECK(sc_v[4][1] == 1051);
    CHECK(sc_v[5][0] == 1050);
    CHECK(sc_v[5][1] == 1064);
    CHECK(sc_v[6][0] == 1050);
    CHECK(sc_v[6][1] == 1065);
    CHECK(sc_v[7][0] == 1036);
    CHECK(sc_v[7][1] == 1037);
    CHECK(sc_v[7][2] == 1050);
    CHECK(sc_v[8][0] == 1036);
    CHECK(sc_v[8][1] == 1049);
    CHECK(sc_v[8][2] == 1050);
    CHECK(sc_v[9][0] == 1037);
    CHECK(sc_v[9][1] == 1050);
    CHECK(sc_v[9][2] == 1051);
    CHECK(sc_v[10][0] == 1049);
    CHECK(sc_v[10][1] == 1050);
    CHECK(sc_v[10][2] == 1064);
    CHECK(sc_v[11][0] == 1050);
    CHECK(sc_v[11][1] == 1051);
    CHECK(sc_v[11][2] == 1065);
    CHECK(sc_v[12][0] == 1050);
    CHECK(sc_v[12][1] == 1064);
    CHECK(sc_v[12][2] == 1065);

    t = m.tuple_from_id(PV, 1781);
    sc_v = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 13);
    CHECK(sc_v[0][0] == 1781);
    CHECK(sc_v[1][0] == 52);
    CHECK(sc_v[1][1] == 1781);
    CHECK(sc_v[2][0] == 53);
    CHECK(sc_v[2][1] == 1781);
    CHECK(sc_v[3][0] == 1765);
    CHECK(sc_v[3][1] == 1781);
    CHECK(sc_v[4][0] == 1766);
    CHECK(sc_v[4][1] == 1781);
    CHECK(sc_v[5][0] == 1780);
    CHECK(sc_v[5][1] == 1781);
    CHECK(sc_v[6][0] == 1781);
    CHECK(sc_v[6][1] == 1782);
    CHECK(sc_v[7][0] == 52);
    CHECK(sc_v[7][1] == 53);
    CHECK(sc_v[7][2] == 1781);
    CHECK(sc_v[8][0] == 52);
    CHECK(sc_v[8][1] == 1781);
    CHECK(sc_v[8][2] == 1782);
    CHECK(sc_v[9][0] == 53);
    CHECK(sc_v[9][1] == 1780);
    CHECK(sc_v[9][2] == 1781);
    CHECK(sc_v[10][0] == 1765);
    CHECK(sc_v[10][1] == 1766);
    CHECK(sc_v[10][2] == 1781);
    CHECK(sc_v[11][0] == 1765);
    CHECK(sc_v[11][1] == 1780);
    CHECK(sc_v[11][2] == 1781);
    CHECK(sc_v[12][0] == 1766);
    CHECK(sc_v[12][1] == 1781);
    CHECK(sc_v[12][2] == 1782);

    t = m.tuple_from_id(PV, 2055);
    sc_v = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 13);
    CHECK(sc_v[0][0] == 2055);
    CHECK(sc_v[1][0] == 2039);
    CHECK(sc_v[1][1] == 2055);
    CHECK(sc_v[2][0] == 2040);
    CHECK(sc_v[2][1] == 2055);
    CHECK(sc_v[3][0] == 2054);
    CHECK(sc_v[3][1] == 2055);
    CHECK(sc_v[4][0] == 2055);
    CHECK(sc_v[4][1] == 2056);
    CHECK(sc_v[5][0] == 2055);
    CHECK(sc_v[5][1] == 2070);
    CHECK(sc_v[6][0] == 2055);
    CHECK(sc_v[6][1] == 2071);
    CHECK(sc_v[7][0] == 2039);
    CHECK(sc_v[7][1] == 2040);
    CHECK(sc_v[7][2] == 2055);
    CHECK(sc_v[8][0] == 2039);
    CHECK(sc_v[8][1] == 2054);
    CHECK(sc_v[8][2] == 2055);
    CHECK(sc_v[9][0] == 2040);
    CHECK(sc_v[9][1] == 2055);
    CHECK(sc_v[9][2] == 2056);
    CHECK(sc_v[10][0] == 2054);
    CHECK(sc_v[10][1] == 2055);
    CHECK(sc_v[10][2] == 2070);
    CHECK(sc_v[11][0] == 2055);
    CHECK(sc_v[11][1] == 2056);
    CHECK(sc_v[11][2] == 2071);
    CHECK(sc_v[12][0] == 2055);
    CHECK(sc_v[12][1] == 2070);
    CHECK(sc_v[12][2] == 2071);

    t = m.edge_tuple_from_vids(277,2246);
    sc_e = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 3);
    CHECK(sc_e[0][0] == 277);
    CHECK(sc_e[0][1] == 2246);
    CHECK(sc_e[1][0] == 276);
    CHECK(sc_e[1][1] == 277);
    CHECK(sc_e[1][2] == 2246);
    CHECK(sc_e[2][0] == 277);
    CHECK(sc_e[2][1] == 2233);
    CHECK(sc_e[2][2] == 2246);

    t = m.edge_tuple_from_vids(4879,4896);
    sc_e = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 3);
    CHECK(sc_e[0][0] == 4879);
    CHECK(sc_e[0][1] == 4896);
    CHECK(sc_e[1][0] == 4878);
    CHECK(sc_e[1][1] == 4879);
    CHECK(sc_e[1][2] == 4896);
    CHECK(sc_e[2][0] == 4879);
    CHECK(sc_e[2][1] == 4896);
    CHECK(sc_e[2][2] == 4897);

    t = m.edge_tuple_from_vids(4124,4125);
    sc_e = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 3);
    CHECK(sc_e[0][0] == 4124);
    CHECK(sc_e[0][1] == 4125);
    CHECK(sc_e[1][0] == 4100);
    CHECK(sc_e[1][1] == 4124);
    CHECK(sc_e[1][2] == 4125);
    CHECK(sc_e[2][0] == 4124);
    CHECK(sc_e[2][1] == 4125);
    CHECK(sc_e[2][2] == 4150);

    t = m.edge_tuple_from_vids(552,553);
    sc_e = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 3);
    CHECK(sc_e[0][0] == 552);
    CHECK(sc_e[0][1] == 553);
    CHECK(sc_e[1][0] == 541);
    CHECK(sc_e[1][1] == 552);
    CHECK(sc_e[1][2] == 553);
    CHECK(sc_e[2][0] == 552);
    CHECK(sc_e[2][1] == 553);
    CHECK(sc_e[2][2] == 565);

    t = m.edge_tuple_from_vids(2113,2128);
    sc_e = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 3);
    CHECK(sc_e[0][0] == 2113);
    CHECK(sc_e[0][1] == 2128);
    CHECK(sc_e[1][0] == 2112);
    CHECK(sc_e[1][1] == 2113);
    CHECK(sc_e[1][2] == 2128);
    CHECK(sc_e[2][0] == 2113);
    CHECK(sc_e[2][1] == 2128);
    CHECK(sc_e[2][2] == 2129);

    t = m.face_tuple_from_vids(31, 496, 503);
    sc_f = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 1);
    CHECK(sc_f[0][0] == 31);
    CHECK(sc_f[0][1] == 496);
    CHECK(sc_f[0][2] == 503);

    t = m.face_tuple_from_vids(1386, 1401, 1402);
    sc_f = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 1);
    CHECK(sc_f[0][0] == 1386);
    CHECK(sc_f[0][1] == 1401);
    CHECK(sc_f[0][2] == 1402);

    t = m.face_tuple_from_vids(1589, 1590, 1605);
    sc_f = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 1);
    CHECK(sc_f[0][0] == 1589);
    CHECK(sc_f[0][1] == 1590);
    CHECK(sc_f[0][2] == 1605);

    t = m.face_tuple_from_vids(10, 482, 486);
    sc_f = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 1);
    CHECK(sc_f[0][0] == 10);
    CHECK(sc_f[0][1] == 482);
    CHECK(sc_f[0][2] == 486);

    t = m.face_tuple_from_vids(3666, 3692, 3693);
    sc_f = get_sorted_sc(m, SimplicialComplex::open_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 1);
    CHECK(sc_f[0][0] == 3666);
    CHECK(sc_f[0][1] == 3692);
    CHECK(sc_f[0][2] == 3693);

}

TEST_CASE("closed_star_circle", "[simplicial_complex][closed_star][2D]")
{
    RowVectors3d V;
    RowVectors3l F;
    std::string name = "/circle.obj";
    std::string path;
    path.append(WMTK_DATA_DIR);
    path.append(name);
    igl::read_triangle_mesh(path, V, F);
    tests::DEBUG_TriMesh m;
    m.initialize(F);

    Tuple t;
    std::vector<std::vector<long>> sc_v, sc_e, sc_f;
    t = m.tuple_from_id(PV, 3062);
    sc_v = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 25);
    CHECK(sc_v[0][0] == 3043);
    CHECK(sc_v[1][0] == 3044);
    CHECK(sc_v[2][0] == 3061);
    CHECK(sc_v[3][0] == 3062);
    CHECK(sc_v[4][0] == 3063);
    CHECK(sc_v[5][0] == 3081);
    CHECK(sc_v[6][0] == 3082);
    CHECK(sc_v[7][0] == 3043);
    CHECK(sc_v[7][1] == 3044);
    CHECK(sc_v[8][0] == 3043);
    CHECK(sc_v[8][1] == 3061);
    CHECK(sc_v[9][0] == 3043);
    CHECK(sc_v[9][1] == 3062);
    CHECK(sc_v[10][0] == 3044);
    CHECK(sc_v[10][1] == 3062);
    CHECK(sc_v[11][0] == 3044);
    CHECK(sc_v[11][1] == 3063);
    CHECK(sc_v[12][0] == 3061);
    CHECK(sc_v[12][1] == 3062);
    CHECK(sc_v[13][0] == 3061);
    CHECK(sc_v[13][1] == 3081);
    CHECK(sc_v[14][0] == 3062);
    CHECK(sc_v[14][1] == 3063);
    CHECK(sc_v[15][0] == 3062);
    CHECK(sc_v[15][1] == 3081);
    CHECK(sc_v[16][0] == 3062);
    CHECK(sc_v[16][1] == 3082);
    CHECK(sc_v[17][0] == 3063);
    CHECK(sc_v[17][1] == 3082);
    CHECK(sc_v[18][0] == 3081);
    CHECK(sc_v[18][1] == 3082);
    CHECK(sc_v[19][0] == 3043);
    CHECK(sc_v[19][1] == 3044);
    CHECK(sc_v[19][2] == 3062);
    CHECK(sc_v[20][0] == 3043);
    CHECK(sc_v[20][1] == 3061);
    CHECK(sc_v[20][2] == 3062);
    CHECK(sc_v[21][0] == 3044);
    CHECK(sc_v[21][1] == 3062);
    CHECK(sc_v[21][2] == 3063);
    CHECK(sc_v[22][0] == 3061);
    CHECK(sc_v[22][1] == 3062);
    CHECK(sc_v[22][2] == 3081);
    CHECK(sc_v[23][0] == 3062);
    CHECK(sc_v[23][1] == 3063);
    CHECK(sc_v[23][2] == 3082);
    CHECK(sc_v[24][0] == 3062);
    CHECK(sc_v[24][1] == 3081);
    CHECK(sc_v[24][2] == 3082);

    t = m.tuple_from_id(PV, 2939);
    sc_v = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 25);
    CHECK(sc_v[0][0] == 2928);
    CHECK(sc_v[1][0] == 2929);
    CHECK(sc_v[2][0] == 2938);
    CHECK(sc_v[3][0] == 2939);
    CHECK(sc_v[4][0] == 2940);
    CHECK(sc_v[5][0] == 2950);
    CHECK(sc_v[6][0] == 2951);
    CHECK(sc_v[7][0] == 2928);
    CHECK(sc_v[7][1] == 2929);
    CHECK(sc_v[8][0] == 2928);
    CHECK(sc_v[8][1] == 2938);
    CHECK(sc_v[9][0] == 2928);
    CHECK(sc_v[9][1] == 2939);
    CHECK(sc_v[10][0] == 2929);
    CHECK(sc_v[10][1] == 2939);
    CHECK(sc_v[11][0] == 2929);
    CHECK(sc_v[11][1] == 2940);
    CHECK(sc_v[12][0] == 2938);
    CHECK(sc_v[12][1] == 2939);
    CHECK(sc_v[13][0] == 2938);
    CHECK(sc_v[13][1] == 2950);
    CHECK(sc_v[14][0] == 2939);
    CHECK(sc_v[14][1] == 2940);
    CHECK(sc_v[15][0] == 2939);
    CHECK(sc_v[15][1] == 2950);
    CHECK(sc_v[16][0] == 2939);
    CHECK(sc_v[16][1] == 2951);
    CHECK(sc_v[17][0] == 2940);
    CHECK(sc_v[17][1] == 2951);
    CHECK(sc_v[18][0] == 2950);
    CHECK(sc_v[18][1] == 2951);
    CHECK(sc_v[19][0] == 2928);
    CHECK(sc_v[19][1] == 2929);
    CHECK(sc_v[19][2] == 2939);
    CHECK(sc_v[20][0] == 2928);
    CHECK(sc_v[20][1] == 2938);
    CHECK(sc_v[20][2] == 2939);
    CHECK(sc_v[21][0] == 2929);
    CHECK(sc_v[21][1] == 2939);
    CHECK(sc_v[21][2] == 2940);
    CHECK(sc_v[22][0] == 2938);
    CHECK(sc_v[22][1] == 2939);
    CHECK(sc_v[22][2] == 2950);
    CHECK(sc_v[23][0] == 2939);
    CHECK(sc_v[23][1] == 2940);
    CHECK(sc_v[23][2] == 2951);
    CHECK(sc_v[24][0] == 2939);
    CHECK(sc_v[24][1] == 2950);
    CHECK(sc_v[24][2] == 2951);

    t = m.tuple_from_id(PV, 100);
    sc_v = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 25);
    CHECK(sc_v[0][0] == 99);
    CHECK(sc_v[1][0] == 100);
    CHECK(sc_v[2][0] == 101);
    CHECK(sc_v[3][0] == 716);
    CHECK(sc_v[4][0] == 718);
    CHECK(sc_v[5][0] == 1436);
    CHECK(sc_v[6][0] == 1437);
    CHECK(sc_v[7][0] == 99);
    CHECK(sc_v[7][1] == 100);
    CHECK(sc_v[8][0] == 99);
    CHECK(sc_v[8][1] == 718);
    CHECK(sc_v[9][0] == 99);
    CHECK(sc_v[9][1] == 1437);
    CHECK(sc_v[10][0] == 100);
    CHECK(sc_v[10][1] == 101);
    CHECK(sc_v[11][0] == 100);
    CHECK(sc_v[11][1] == 716);
    CHECK(sc_v[12][0] == 100);
    CHECK(sc_v[12][1] == 718);
    CHECK(sc_v[13][0] == 100);
    CHECK(sc_v[13][1] == 1436);
    CHECK(sc_v[14][0] == 100);
    CHECK(sc_v[14][1] == 1437);
    CHECK(sc_v[15][0] == 101);
    CHECK(sc_v[15][1] == 716);
    CHECK(sc_v[16][0] == 101);
    CHECK(sc_v[16][1] == 1436);
    CHECK(sc_v[17][0] == 716);
    CHECK(sc_v[17][1] == 718);
    CHECK(sc_v[18][0] == 1436);
    CHECK(sc_v[18][1] == 1437);
    CHECK(sc_v[19][0] == 99);
    CHECK(sc_v[19][1] == 100);
    CHECK(sc_v[19][2] == 718);
    CHECK(sc_v[20][0] == 99);
    CHECK(sc_v[20][1] == 100);
    CHECK(sc_v[20][2] == 1437);
    CHECK(sc_v[21][0] == 100);
    CHECK(sc_v[21][1] == 101);
    CHECK(sc_v[21][2] == 716);
    CHECK(sc_v[22][0] == 100);
    CHECK(sc_v[22][1] == 101);
    CHECK(sc_v[22][2] == 1436);
    CHECK(sc_v[23][0] == 100);
    CHECK(sc_v[23][1] == 716);
    CHECK(sc_v[23][2] == 718);
    CHECK(sc_v[24][0] == 100);
    CHECK(sc_v[24][1] == 1436);
    CHECK(sc_v[24][2] == 1437);

    t = m.tuple_from_id(PV, 2801);
    sc_v = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 25);
    CHECK(sc_v[0][0] == 2785);
    CHECK(sc_v[1][0] == 2786);
    CHECK(sc_v[2][0] == 2800);
    CHECK(sc_v[3][0] == 2801);
    CHECK(sc_v[4][0] == 2802);
    CHECK(sc_v[5][0] == 2816);
    CHECK(sc_v[6][0] == 2817);
    CHECK(sc_v[7][0] == 2785);
    CHECK(sc_v[7][1] == 2786);
    CHECK(sc_v[8][0] == 2785);
    CHECK(sc_v[8][1] == 2800);
    CHECK(sc_v[9][0] == 2785);
    CHECK(sc_v[9][1] == 2801);
    CHECK(sc_v[10][0] == 2786);
    CHECK(sc_v[10][1] == 2801);
    CHECK(sc_v[11][0] == 2786);
    CHECK(sc_v[11][1] == 2802);
    CHECK(sc_v[12][0] == 2800);
    CHECK(sc_v[12][1] == 2801);
    CHECK(sc_v[13][0] == 2800);
    CHECK(sc_v[13][1] == 2816);
    CHECK(sc_v[14][0] == 2801);
    CHECK(sc_v[14][1] == 2802);
    CHECK(sc_v[15][0] == 2801);
    CHECK(sc_v[15][1] == 2816);
    CHECK(sc_v[16][0] == 2801);
    CHECK(sc_v[16][1] == 2817);
    CHECK(sc_v[17][0] == 2802);
    CHECK(sc_v[17][1] == 2817);
    CHECK(sc_v[18][0] == 2816);
    CHECK(sc_v[18][1] == 2817);
    CHECK(sc_v[19][0] == 2785);
    CHECK(sc_v[19][1] == 2786);
    CHECK(sc_v[19][2] == 2801);
    CHECK(sc_v[20][0] == 2785);
    CHECK(sc_v[20][1] == 2800);
    CHECK(sc_v[20][2] == 2801);
    CHECK(sc_v[21][0] == 2786);
    CHECK(sc_v[21][1] == 2801);
    CHECK(sc_v[21][2] == 2802);
    CHECK(sc_v[22][0] == 2800);
    CHECK(sc_v[22][1] == 2801);
    CHECK(sc_v[22][2] == 2816);
    CHECK(sc_v[23][0] == 2801);
    CHECK(sc_v[23][1] == 2802);
    CHECK(sc_v[23][2] == 2817);
    CHECK(sc_v[24][0] == 2801);
    CHECK(sc_v[24][1] == 2816);
    CHECK(sc_v[24][2] == 2817);

    t = m.tuple_from_id(PV, 4947);
    sc_v = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 25);
    CHECK(sc_v[0][0] == 4926);
    CHECK(sc_v[1][0] == 4927);
    CHECK(sc_v[2][0] == 4946);
    CHECK(sc_v[3][0] == 4947);
    CHECK(sc_v[4][0] == 4948);
    CHECK(sc_v[5][0] == 4968);
    CHECK(sc_v[6][0] == 4969);
    CHECK(sc_v[7][0] == 4926);
    CHECK(sc_v[7][1] == 4927);
    CHECK(sc_v[8][0] == 4926);
    CHECK(sc_v[8][1] == 4946);
    CHECK(sc_v[9][0] == 4926);
    CHECK(sc_v[9][1] == 4947);
    CHECK(sc_v[10][0] == 4927);
    CHECK(sc_v[10][1] == 4947);
    CHECK(sc_v[11][0] == 4927);
    CHECK(sc_v[11][1] == 4948);
    CHECK(sc_v[12][0] == 4946);
    CHECK(sc_v[12][1] == 4947);
    CHECK(sc_v[13][0] == 4946);
    CHECK(sc_v[13][1] == 4968);
    CHECK(sc_v[14][0] == 4947);
    CHECK(sc_v[14][1] == 4948);
    CHECK(sc_v[15][0] == 4947);
    CHECK(sc_v[15][1] == 4968);
    CHECK(sc_v[16][0] == 4947);
    CHECK(sc_v[16][1] == 4969);
    CHECK(sc_v[17][0] == 4948);
    CHECK(sc_v[17][1] == 4969);
    CHECK(sc_v[18][0] == 4968);
    CHECK(sc_v[18][1] == 4969);
    CHECK(sc_v[19][0] == 4926);
    CHECK(sc_v[19][1] == 4927);
    CHECK(sc_v[19][2] == 4947);
    CHECK(sc_v[20][0] == 4926);
    CHECK(sc_v[20][1] == 4946);
    CHECK(sc_v[20][2] == 4947);
    CHECK(sc_v[21][0] == 4927);
    CHECK(sc_v[21][1] == 4947);
    CHECK(sc_v[21][2] == 4948);
    CHECK(sc_v[22][0] == 4946);
    CHECK(sc_v[22][1] == 4947);
    CHECK(sc_v[22][2] == 4968);
    CHECK(sc_v[23][0] == 4947);
    CHECK(sc_v[23][1] == 4948);
    CHECK(sc_v[23][2] == 4969);
    CHECK(sc_v[24][0] == 4947);
    CHECK(sc_v[24][1] == 4968);
    CHECK(sc_v[24][2] == 4969);

    t = m.edge_tuple_from_vids(3759,3760);
    sc_e = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 11);
    CHECK(sc_e[0][0] == 3731);
    CHECK(sc_e[1][0] == 3759);
    CHECK(sc_e[2][0] == 3760);
    CHECK(sc_e[3][0] == 3789);
    CHECK(sc_e[4][0] == 3731);
    CHECK(sc_e[4][1] == 3759);
    CHECK(sc_e[5][0] == 3731);
    CHECK(sc_e[5][1] == 3760);
    CHECK(sc_e[6][0] == 3759);
    CHECK(sc_e[6][1] == 3760);
    CHECK(sc_e[7][0] == 3759);
    CHECK(sc_e[7][1] == 3789);
    CHECK(sc_e[8][0] == 3760);
    CHECK(sc_e[8][1] == 3789);
    CHECK(sc_e[9][0] == 3731);
    CHECK(sc_e[9][1] == 3759);
    CHECK(sc_e[9][2] == 3760);
    CHECK(sc_e[10][0] == 3759);
    CHECK(sc_e[10][1] == 3760);
    CHECK(sc_e[10][2] == 3789);

    t = m.edge_tuple_from_vids(884,885);
    sc_e = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 11);
    CHECK(sc_e[0][0] == 875);
    CHECK(sc_e[1][0] == 884);
    CHECK(sc_e[2][0] == 885);
    CHECK(sc_e[3][0] == 895);
    CHECK(sc_e[4][0] == 875);
    CHECK(sc_e[4][1] == 884);
    CHECK(sc_e[5][0] == 875);
    CHECK(sc_e[5][1] == 885);
    CHECK(sc_e[6][0] == 884);
    CHECK(sc_e[6][1] == 885);
    CHECK(sc_e[7][0] == 884);
    CHECK(sc_e[7][1] == 895);
    CHECK(sc_e[8][0] == 885);
    CHECK(sc_e[8][1] == 895);
    CHECK(sc_e[9][0] == 875);
    CHECK(sc_e[9][1] == 884);
    CHECK(sc_e[9][2] == 885);
    CHECK(sc_e[10][0] == 884);
    CHECK(sc_e[10][1] == 885);
    CHECK(sc_e[10][2] == 895);

    t = m.edge_tuple_from_vids(4074,4075);
    sc_e = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 11);
    CHECK(sc_e[0][0] == 4052);
    CHECK(sc_e[1][0] == 4074);
    CHECK(sc_e[2][0] == 4075);
    CHECK(sc_e[3][0] == 4098);
    CHECK(sc_e[4][0] == 4052);
    CHECK(sc_e[4][1] == 4074);
    CHECK(sc_e[5][0] == 4052);
    CHECK(sc_e[5][1] == 4075);
    CHECK(sc_e[6][0] == 4074);
    CHECK(sc_e[6][1] == 4075);
    CHECK(sc_e[7][0] == 4074);
    CHECK(sc_e[7][1] == 4098);
    CHECK(sc_e[8][0] == 4075);
    CHECK(sc_e[8][1] == 4098);
    CHECK(sc_e[9][0] == 4052);
    CHECK(sc_e[9][1] == 4074);
    CHECK(sc_e[9][2] == 4075);
    CHECK(sc_e[10][0] == 4074);
    CHECK(sc_e[10][1] == 4075);
    CHECK(sc_e[10][2] == 4098);

    t = m.edge_tuple_from_vids(4644,4671);
    sc_e = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 11);
    CHECK(sc_e[0][0] == 4643);
    CHECK(sc_e[1][0] == 4644);
    CHECK(sc_e[2][0] == 4671);
    CHECK(sc_e[3][0] == 4672);
    CHECK(sc_e[4][0] == 4643);
    CHECK(sc_e[4][1] == 4644);
    CHECK(sc_e[5][0] == 4643);
    CHECK(sc_e[5][1] == 4671);
    CHECK(sc_e[6][0] == 4644);
    CHECK(sc_e[6][1] == 4671);
    CHECK(sc_e[7][0] == 4644);
    CHECK(sc_e[7][1] == 4672);
    CHECK(sc_e[8][0] == 4671);
    CHECK(sc_e[8][1] == 4672);
    CHECK(sc_e[9][0] == 4643);
    CHECK(sc_e[9][1] == 4644);
    CHECK(sc_e[9][2] == 4671);
    CHECK(sc_e[10][0] == 4644);
    CHECK(sc_e[10][1] == 4671);
    CHECK(sc_e[10][2] == 4672);

    t = m.edge_tuple_from_vids(2357,2372);
    sc_e = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 11);
    CHECK(sc_e[0][0] == 2356);
    CHECK(sc_e[1][0] == 2357);
    CHECK(sc_e[2][0] == 2372);
    CHECK(sc_e[3][0] == 2373);
    CHECK(sc_e[4][0] == 2356);
    CHECK(sc_e[4][1] == 2357);
    CHECK(sc_e[5][0] == 2356);
    CHECK(sc_e[5][1] == 2372);
    CHECK(sc_e[6][0] == 2357);
    CHECK(sc_e[6][1] == 2372);
    CHECK(sc_e[7][0] == 2357);
    CHECK(sc_e[7][1] == 2373);
    CHECK(sc_e[8][0] == 2372);
    CHECK(sc_e[8][1] == 2373);
    CHECK(sc_e[9][0] == 2356);
    CHECK(sc_e[9][1] == 2357);
    CHECK(sc_e[9][2] == 2372);
    CHECK(sc_e[10][0] == 2357);
    CHECK(sc_e[10][1] == 2372);
    CHECK(sc_e[10][2] == 2373);

    t = m.face_tuple_from_vids(175, 1291, 1307);
    sc_f = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 7);
    CHECK(sc_f[0][0] == 175);
    CHECK(sc_f[1][0] == 1291);
    CHECK(sc_f[2][0] == 1307);
    CHECK(sc_f[3][0] == 175);
    CHECK(sc_f[3][1] == 1291);
    CHECK(sc_f[4][0] == 175);
    CHECK(sc_f[4][1] == 1307);
    CHECK(sc_f[5][0] == 1291);
    CHECK(sc_f[5][1] == 1307);
    CHECK(sc_f[6][0] == 175);
    CHECK(sc_f[6][1] == 1291);
    CHECK(sc_f[6][2] == 1307);

    t = m.face_tuple_from_vids(989, 997, 998);
    sc_f = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 7);
    CHECK(sc_f[0][0] == 989);
    CHECK(sc_f[1][0] == 997);
    CHECK(sc_f[2][0] == 998);
    CHECK(sc_f[3][0] == 989);
    CHECK(sc_f[3][1] == 997);
    CHECK(sc_f[4][0] == 989);
    CHECK(sc_f[4][1] == 998);
    CHECK(sc_f[5][0] == 997);
    CHECK(sc_f[5][1] == 998);
    CHECK(sc_f[6][0] == 989);
    CHECK(sc_f[6][1] == 997);
    CHECK(sc_f[6][2] == 998);

    t = m.face_tuple_from_vids(46, 47, 632);
    sc_f = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 7);
    CHECK(sc_f[0][0] == 46);
    CHECK(sc_f[1][0] == 47);
    CHECK(sc_f[2][0] == 632);
    CHECK(sc_f[3][0] == 46);
    CHECK(sc_f[3][1] == 47);
    CHECK(sc_f[4][0] == 46);
    CHECK(sc_f[4][1] == 632);
    CHECK(sc_f[5][0] == 47);
    CHECK(sc_f[5][1] == 632);
    CHECK(sc_f[6][0] == 46);
    CHECK(sc_f[6][1] == 47);
    CHECK(sc_f[6][2] == 632);

    t = m.face_tuple_from_vids(1186, 1187, 1202);
    sc_f = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 7);
    CHECK(sc_f[0][0] == 1186);
    CHECK(sc_f[1][0] == 1187);
    CHECK(sc_f[2][0] == 1202);
    CHECK(sc_f[3][0] == 1186);
    CHECK(sc_f[3][1] == 1187);
    CHECK(sc_f[4][0] == 1186);
    CHECK(sc_f[4][1] == 1202);
    CHECK(sc_f[5][0] == 1187);
    CHECK(sc_f[5][1] == 1202);
    CHECK(sc_f[6][0] == 1186);
    CHECK(sc_f[6][1] == 1187);
    CHECK(sc_f[6][2] == 1202);

    t = m.face_tuple_from_vids(1268, 1269, 1284);
    sc_f = get_sorted_sc(m, SimplicialComplex::closed_star(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 7);
    CHECK(sc_f[0][0] == 1268);
    CHECK(sc_f[1][0] == 1269);
    CHECK(sc_f[2][0] == 1284);
    CHECK(sc_f[3][0] == 1268);
    CHECK(sc_f[3][1] == 1269);
    CHECK(sc_f[4][0] == 1268);
    CHECK(sc_f[4][1] == 1284);
    CHECK(sc_f[5][0] == 1269);
    CHECK(sc_f[5][1] == 1284);
    CHECK(sc_f[6][0] == 1268);
    CHECK(sc_f[6][1] == 1269);
    CHECK(sc_f[6][2] == 1284);

}

TEST_CASE("link_circle", "[simplicial_complex][link][2D]")
{
    RowVectors3d V;
    RowVectors3l F;
    std::string name = "/circle.obj";
    std::string path;
    path.append(WMTK_DATA_DIR);
    path.append(name);
    igl::read_triangle_mesh(path, V, F);
    tests::DEBUG_TriMesh m;
    m.initialize(F);

    Tuple t;
    std::vector<std::vector<long>> sc_v, sc_e, sc_f;
    t = m.tuple_from_id(PV, 3342);
    sc_v = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 12);
    CHECK(sc_v[0][0] == 384);
    CHECK(sc_v[1][0] == 385);
    CHECK(sc_v[2][0] == 3341);
    CHECK(sc_v[3][0] == 3343);
    CHECK(sc_v[4][0] == 3344);
    CHECK(sc_v[5][0] == 3345);
    CHECK(sc_v[6][0] == 384);
    CHECK(sc_v[6][1] == 385);
    CHECK(sc_v[7][0] == 384);
    CHECK(sc_v[7][1] == 3341);
    CHECK(sc_v[8][0] == 385);
    CHECK(sc_v[8][1] == 3344);
    CHECK(sc_v[9][0] == 3341);
    CHECK(sc_v[9][1] == 3343);
    CHECK(sc_v[10][0] == 3343);
    CHECK(sc_v[10][1] == 3345);
    CHECK(sc_v[11][0] == 3344);
    CHECK(sc_v[11][1] == 3345);

    t = m.tuple_from_id(PV, 3467);
    sc_v = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 12);
    CHECK(sc_v[0][0] == 3451);
    CHECK(sc_v[1][0] == 3452);
    CHECK(sc_v[2][0] == 3466);
    CHECK(sc_v[3][0] == 3468);
    CHECK(sc_v[4][0] == 3483);
    CHECK(sc_v[5][0] == 3484);
    CHECK(sc_v[6][0] == 3451);
    CHECK(sc_v[6][1] == 3452);
    CHECK(sc_v[7][0] == 3451);
    CHECK(sc_v[7][1] == 3466);
    CHECK(sc_v[8][0] == 3452);
    CHECK(sc_v[8][1] == 3468);
    CHECK(sc_v[9][0] == 3466);
    CHECK(sc_v[9][1] == 3483);
    CHECK(sc_v[10][0] == 3468);
    CHECK(sc_v[10][1] == 3484);
    CHECK(sc_v[11][0] == 3483);
    CHECK(sc_v[11][1] == 3484);

    t = m.tuple_from_id(PV, 462);
    sc_v = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 12);
    CHECK(sc_v[0][0] == 461);
    CHECK(sc_v[1][0] == 463);
    CHECK(sc_v[2][0] == 4407);
    CHECK(sc_v[3][0] == 4424);
    CHECK(sc_v[4][0] == 4888);
    CHECK(sc_v[5][0] == 4906);
    CHECK(sc_v[6][0] == 461);
    CHECK(sc_v[6][1] == 4407);
    CHECK(sc_v[7][0] == 461);
    CHECK(sc_v[7][1] == 4888);
    CHECK(sc_v[8][0] == 463);
    CHECK(sc_v[8][1] == 4424);
    CHECK(sc_v[9][0] == 463);
    CHECK(sc_v[9][1] == 4906);
    CHECK(sc_v[10][0] == 4407);
    CHECK(sc_v[10][1] == 4424);
    CHECK(sc_v[11][0] == 4888);
    CHECK(sc_v[11][1] == 4906);

    t = m.tuple_from_id(PV, 2714);
    sc_v = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 12);
    CHECK(sc_v[0][0] == 2698);
    CHECK(sc_v[1][0] == 2699);
    CHECK(sc_v[2][0] == 2713);
    CHECK(sc_v[3][0] == 2715);
    CHECK(sc_v[4][0] == 2729);
    CHECK(sc_v[5][0] == 2730);
    CHECK(sc_v[6][0] == 2698);
    CHECK(sc_v[6][1] == 2699);
    CHECK(sc_v[7][0] == 2698);
    CHECK(sc_v[7][1] == 2713);
    CHECK(sc_v[8][0] == 2699);
    CHECK(sc_v[8][1] == 2715);
    CHECK(sc_v[9][0] == 2713);
    CHECK(sc_v[9][1] == 2729);
    CHECK(sc_v[10][0] == 2715);
    CHECK(sc_v[10][1] == 2730);
    CHECK(sc_v[11][0] == 2729);
    CHECK(sc_v[11][1] == 2730);

    t = m.tuple_from_id(PV, 835);
    sc_v = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PV, t)).get_simplex_vector());
    REQUIRE(sc_v.size() == 7);
    CHECK(sc_v[0][0] == 86);
    CHECK(sc_v[1][0] == 87);
    CHECK(sc_v[2][0] == 820);
    CHECK(sc_v[3][0] == 834);
    CHECK(sc_v[4][0] == 86);
    CHECK(sc_v[4][1] == 87);
    CHECK(sc_v[5][0] == 87);
    CHECK(sc_v[5][1] == 820);
    CHECK(sc_v[6][0] == 820);
    CHECK(sc_v[6][1] == 834);

    t = m.edge_tuple_from_vids(207,3796);
    sc_e = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 2);
    CHECK(sc_e[0][0] == 206);
    CHECK(sc_e[1][0] == 3795);

    t = m.edge_tuple_from_vids(1516,1530);
    sc_e = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 2);
    CHECK(sc_e[0][0] == 1517);
    CHECK(sc_e[1][0] == 1529);

    t = m.edge_tuple_from_vids(3508,3509);
    sc_e = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 2);
    CHECK(sc_e[0][0] == 3491);
    CHECK(sc_e[1][0] == 3527);

    t = m.edge_tuple_from_vids(4182,4209);
    sc_e = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 2);
    CHECK(sc_e[0][0] == 4181);
    CHECK(sc_e[1][0] == 4210);

    t = m.edge_tuple_from_vids(112,113);
    sc_e = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PE, t)).get_simplex_vector());
    REQUIRE(sc_e.size() == 2);
    CHECK(sc_e[0][0] == 891);
    CHECK(sc_e[1][0] == 2505);

    t = m.face_tuple_from_vids(3560, 3561, 3582);
    sc_f = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 0);

    t = m.face_tuple_from_vids(2351, 2352, 2367);
    sc_f = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 0);

    t = m.face_tuple_from_vids(1231, 1232, 1247);
    sc_f = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 0);

    t = m.face_tuple_from_vids(3863, 3864, 3875);
    sc_f = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 0);

    t = m.face_tuple_from_vids(653, 654, 665);
    sc_f = get_sorted_sc(m, SimplicialComplex::link(m, Simplex(PF, t)).get_simplex_vector());
    REQUIRE(sc_f.size() == 0);

}
