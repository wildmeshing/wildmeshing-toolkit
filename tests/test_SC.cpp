// test code for SC
// Psudo code now

// #include "SimplicialComplex.hpp"
#include <catch2/catch.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

using namespace wmtk;

TEST_CASE("simplex_comparison", "[SC]")
{
    TriMesh m;
    {
        RowVectors3l tris(2, 3);
        tris << 0, 1, 2, 2, 1, 3;
        m.initialize(tris);
    }

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 4);
        for (const auto& t : vertices) {
            const Simplex s0(PrimitiveType::Vertex, t);
            const Simplex s1(PrimitiveType::Vertex, m.switch_tuple(t, PrimitiveType::Edge));
            CHECK_FALSE(m.simplex_is_less(s0, s1));
            CHECK_FALSE(m.simplex_is_less(s1, s0));
            if (m.is_boundary(t)) {
                continue;
            }
            const Simplex s2(PrimitiveType::Vertex, m.switch_tuple(t, PrimitiveType::Face));
            CHECK_FALSE(m.simplex_is_less(s0, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s0));
            CHECK_FALSE(m.simplex_is_less(s1, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s1));
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 5);
        for (const auto& t : edges) {
            const Simplex s0(PrimitiveType::Edge, t);
            const Simplex s1(PrimitiveType::Edge, m.switch_tuple(t, PrimitiveType::Vertex));
            CHECK_FALSE(m.simplex_is_less(s0, s1));
            CHECK_FALSE(m.simplex_is_less(s1, s0));
            if (m.is_boundary(t)) {
                continue;
            }
            const Simplex s2(PrimitiveType::Edge, m.switch_tuple(t, PrimitiveType::Face));
            CHECK_FALSE(m.simplex_is_less(s0, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s0));
            CHECK_FALSE(m.simplex_is_less(s1, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s1));
        }
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Face);
        REQUIRE(faces.size() == 2);
        for (const auto& t : faces) {
            const Simplex s0(PrimitiveType::Face, t);
            const Simplex s1(PrimitiveType::Face, m.switch_tuple(t, PrimitiveType::Vertex));
            CHECK_FALSE(m.simplex_is_less(s0, s1));
            CHECK_FALSE(m.simplex_is_less(s1, s0));
            const Simplex s2(PrimitiveType::Face, m.switch_tuple(t, PrimitiveType::Edge));
            CHECK_FALSE(m.simplex_is_less(s0, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s0));
            CHECK_FALSE(m.simplex_is_less(s1, s2));
            CHECK_FALSE(m.simplex_is_less(s2, s1));
        }
    }
}

TEST_CASE("link-case1", "[SC][link]")
{
    RowVectors3l F(3, 3);
    F << 0, 3, 2, 0, 1, 3, 1, 2, 3; // 3 Faces

    // dump it to (Tri)Mesh
    TriMesh m;
    m.initialize(F);

    // get the tuple point to V(0), E(01), F(013)
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);


    SimplicialComplex lnk_0 = SimplicialComplex::link(Simplex(PrimitiveType::Vertex, t), m);
    // SimplicialComplex lnk_1 = SimplicialComplex::link(Simplex(PrimitiveType::Vertex,
    // m.switch_tuple(t,PrimitiveType::Vertex)), m);


    // SimplicialComplex lhs = SimplicialComplex::get_intersection(lnk_0, lnk_1);
    SimplicialComplex lnk_01 = SimplicialComplex::link(Simplex(PrimitiveType::Edge, t), m);
    SimplicialComplex lnk_10 = SimplicialComplex::link(
        Simplex(PrimitiveType::Edge, m.switch_tuple(t, PrimitiveType::Edge)),
        m);

    std::cout << "lnk_0 Vertex size = " << lnk_0.get_simplices(PrimitiveType::Vertex).size()
              << std::endl;
    std::cout << "lnk_0 Edge size = " << lnk_0.get_simplices(PrimitiveType::Edge).size()
              << std::endl;
    std::cout << "lnk_0 Face size = " << lnk_0.get_simplices(PrimitiveType::Face).size()
              << std::endl;

    REQUIRE(lnk_0.get_simplices().size() == 5);
    // REQUIRE(lnk_1.get_simplices().size() == 5);


    std::cout << "lnk_01 Vertex size = " << lnk_01.get_simplices(PrimitiveType::Vertex).size()
              << std::endl;
    std::cout << "lnk_01 Edge size = " << lnk_01.get_simplices(PrimitiveType::Edge).size()
              << std::endl;
    std::cout << "lnk_01 Face size = " << lnk_01.get_simplices(PrimitiveType::Face).size()
              << std::endl;
    REQUIRE(lnk_01.get_simplices().size() == 1);
    // REQUIRE(lhs.get_simplices().size() == 3);

    std::cout << "lnk_10 Vertex size = " << lnk_10.get_simplices(PrimitiveType::Vertex).size()
              << std::endl;
    std::cout << "lnk_10 Edge size = " << lnk_10.get_simplices(PrimitiveType::Edge).size()
              << std::endl;
    std::cout << "lnk_10 Face size = " << lnk_10.get_simplices(PrimitiveType::Face).size()
              << std::endl;
    REQUIRE(lnk_01 == lnk_10);

    // REQUIRE(SimplicialComplex::link_cond(t, m) == false);
}


TEST_CASE("link-case2", "[SC][link]")
{
    RowVectors3l F(4, 3);
    F << 0, 3, 1, 0, 1, 2, 0, 2, 4, 2, 1, 5; // 4 Faces

    // dump it to (Tri)Mesh
    TriMesh m;
    m.initialize(F);

    // get the tuple point to V(0), E(01), F(012)
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);

    SimplicialComplex lnk_0 = SimplicialComplex::link(Simplex(PrimitiveType::Vertex, t), m);
    // SimplicialComplex lnk_1 = SimplicialComplex::link(Simplex(PrimitiveType::Vertex,
    // m.switch_tuple(t,PrimitiveType::Vertex)), m);


    // SimplicialComplex lhs = SimplicialComplex::get_intersection(lnk_0, lnk_1);
    // SimplicialComplex lnk_01 = SimplicialComplex::link(Simplex(PrimitiveType::Edge, t), m);
    // SimplicialComplex lnk_10 = SimplicialComplex::link(Simplex(PrimitiveType::Edge,
    // m.switch_tuple(t,PrimitiveType::Edge)), m);


    REQUIRE(lnk_0.get_simplices().size() == 7);
    // REQUIRE(lnk_1.get_simplices().size() == 7);
    // REQUIRE(lnk_01.get_simplices().size() == 2);

    // REQUIRE(lhs == lnk_01);
    // REQUIRE(lnk_01 == lnk_10);

    // REQUIRE(link_cond(t, m) == true);
}

TEST_CASE("k-ring test", "[SC][k-ring]")
{
    RowVectors3l F(4, 3);
    F << 0, 3, 1, 0, 1, 2, 0, 2, 4, 2, 1, 5; // 4 Faces

    // dump it to (Tri)Mesh
    TriMesh m;
    m.initialize(F);

    // get the tuple point to V(3)
    long hash = 0;
    Tuple t(1, 0, -1, 0, hash);

    // REQUIRE(vertex_one_ring(t, m).size() == 2);
    // REQUIRE(k_ring(t, m, 1).size() == 2);
    // REQUIRE(k_ring(t, m, 2).size() == 6);
    // REQUIRE(k_ring(t, m, 3).size() == 6);
}

TEST_CASE("star", "[SC][open star]")
{
    RowVectors3l F(4, 3);
    F << 0, 3, 1, 0, 1, 2, 0, 2, 4, 2, 1, 5; // 4 Faces

    // dump it to (Tri)Mesh
    TriMesh m;
    m.initialize(F);

    // get the tuple point to V(0), E(01), F(012)
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);


    // SimplicialComplex sc_v = open_star(Simlex(t, 0), m);
    // REQUIRE(sc_v.get_size() == 8);

    // SimplicialComplex sc_e = open_star(Simlex(t, 1), m);
    // REQUIRE(sc_e.get_size() == 3);

    // SimplicialComplex sc_1 = open_star(Simlex(t, 2), m);
    // REQUIRE(sc_f.get_size() == 1);
}
