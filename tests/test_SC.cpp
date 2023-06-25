// test code for SC
// Psudo code now

// #include "SimplicialComplex.hpp"
#include <catch2/catch.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

using namespace wmtk;


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
    SimplicialComplex lnk_1 = SimplicialComplex::link(Simplex(PrimitiveType::Vertex, m.switch_tuple(t,PrimitiveType::Vertex)), m);
    
    
    SimplicialComplex lhs = SimplicialComplex::get_intersection(lnk_0, lnk_1);
    SimplicialComplex lnk_01 = SimplicialComplex::link(Simplex(PrimitiveType::Edge, t), m);
    
    SimplicialComplex lnk_10 = SimplicialComplex::link(Simplex(PrimitiveType::Edge, m.switch_tuple(t,PrimitiveType::Vertex)), m);
    
    std::cout << "lnk_0 Vertex size = " << lnk_0.get_simplices(PrimitiveType::Vertex).size() << std::endl;
    std::cout << "lnk_0 Edge size = " << lnk_0.get_simplices(PrimitiveType::Edge).size() << std::endl;
    std::cout << "lnk_0 Face size = " << lnk_0.get_simplices(PrimitiveType::Face).size() << std::endl;
    
    REQUIRE(lnk_0.get_simplices().size() == 5);
    REQUIRE(lnk_1.get_simplices().size() == 5);


    std::cout << "lnk_01 Vertex size = " << lnk_01.get_simplices(PrimitiveType::Vertex).size() << std::endl;
    std::cout << "lnk_01 Edge size = " << lnk_01.get_simplices(PrimitiveType::Edge).size() << std::endl;
    std::cout << "lnk_01 Face size = " << lnk_01.get_simplices(PrimitiveType::Face).size() << std::endl;
    REQUIRE(lnk_01.get_simplices().size() == 1);
    REQUIRE(lhs.get_simplices().size() == 3);

    std::cout << "lnk_10 Vertex size = " << lnk_10.get_simplices(PrimitiveType::Vertex).size() << std::endl;
    std::cout << "lnk_10 Edge size = " << lnk_10.get_simplices(PrimitiveType::Edge).size() << std::endl;
    std::cout << "lnk_10 Face size = " << lnk_10.get_simplices(PrimitiveType::Face).size() << std::endl;
    REQUIRE(lnk_01 == lnk_10);

    REQUIRE(SimplicialComplex::link_cond(t, m) == false);
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
    SimplicialComplex lnk_1 = SimplicialComplex::link(Simplex(PrimitiveType::Vertex, m.switch_tuple(t,PrimitiveType::Vertex)), m);
    
    
    SimplicialComplex lhs = SimplicialComplex::get_intersection(lnk_0, lnk_1);
    SimplicialComplex lnk_01 = SimplicialComplex::link(Simplex(PrimitiveType::Edge, t), m);
    SimplicialComplex lnk_10 = SimplicialComplex::link(Simplex(PrimitiveType::Edge, m.switch_tuple(t,PrimitiveType::Vertex)), m);


    REQUIRE(lnk_0.get_simplices().size() == 7);
    REQUIRE(lnk_1.get_simplices().size() == 7);
    REQUIRE(lnk_01.get_simplices().size() == 2);

    REQUIRE(lhs == lnk_01);
    REQUIRE(lnk_01 == lnk_10);

    REQUIRE(SimplicialComplex::link_cond(t, m) == true);
}

TEST_CASE("k-ring", "[SC][k-ring]")
{
    RowVectors3l F(4, 3);
    F << 0, 3, 1, 0, 1, 2, 0, 2, 4, 2, 1, 5; // 4 Faces

    // dump it to (Tri)Mesh
    TriMesh m;
    m.initialize(F);

    // get the tuple point to V(3)
    long hash = 0;
    Tuple t(1, 0, -1, 0, hash);

    auto ret1 = SimplicialComplex::vertex_one_ring(t, m);
    REQUIRE(ret1.size() == 2);
    auto ret2 = SimplicialComplex::k_ring(t, m, 1);
    REQUIRE(ret2.size() == 2);
    auto ret3 = SimplicialComplex::k_ring(t, m, 2);
    REQUIRE(ret3.size() == 6);
    auto ret4 = SimplicialComplex::k_ring(t, m, 3);
    REQUIRE(ret4.size() == 6);
}

TEST_CASE("open_star", "[SC][star]")
{
    RowVectors3l F(4, 3);
    F << 0, 3, 1, 0, 1, 2, 0, 2, 4, 2, 1, 5; // 4 Faces

    // dump it to (Tri)Mesh
    TriMesh m;
    m.initialize(F);

    // get the tuple point to V(0), E(01), F(012)
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);


    SimplicialComplex sc_v = SimplicialComplex::open_star(Simplex(PrimitiveType::Vertex, t), m);
    REQUIRE(sc_v.get_simplices().size() == 8);

    SimplicialComplex sc_e = SimplicialComplex::open_star(Simplex(PrimitiveType::Edge, t), m);
    REQUIRE(sc_e.get_simplices().size() == 3);

    SimplicialComplex sc_f = SimplicialComplex::open_star(Simplex(PrimitiveType::Face, t), m);
    REQUIRE(sc_f.get_simplices().size() == 1);
}

TEST_CASE("closed_star", "[SC][star]")
{
    RowVectors3l F(4, 3);
    F << 0, 3, 1, 0, 1, 2, 0, 2, 4, 2, 1, 5; // 4 Faces

    // dump it to (Tri)Mesh
    TriMesh m;
    m.initialize(F);

    // get the tuple point to V(0), E(01), F(012)
    long hash = 0;
    Tuple t(0, 2, -1, 1, hash);


    SimplicialComplex sc_v = SimplicialComplex::closed_star(Simplex(PrimitiveType::Vertex, t), m);
    std::cout << "sc_v Vertex size = " << sc_v.get_simplices(PrimitiveType::Vertex).size() << std::endl;
    std::cout << "sc_v Edge size = " << sc_v.get_simplices(PrimitiveType::Edge).size() << std::endl;
    std::cout << "sc_v Face size = " << sc_v.get_simplices(PrimitiveType::Face).size() << std::endl;
    REQUIRE(sc_v.get_simplices().size() == 15);

    SimplicialComplex sc_e = SimplicialComplex::closed_star(Simplex(PrimitiveType::Edge, t), m);
    REQUIRE(sc_e.get_simplices().size() == 11);

    SimplicialComplex sc_f = SimplicialComplex::closed_star(Simplex(PrimitiveType::Face, t), m);
    REQUIRE(sc_f.get_simplices().size() == 7);
}