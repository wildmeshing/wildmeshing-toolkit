#pragma once
#include <catch2/catch_test_macros.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/energy/AMIPS.hpp>
#include <wmtk/energy/TriMeshValenceEnergy.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::energy;
using namespace wmtk::tests;
TEST_CASE("energy_valence")
{
    //    0---1---2
    //   /0\1/2\3/4\ .
    //  3---4---5---6
    //   \5/6\7/  .
    //    7---8
    const DEBUG_TriMesh example_mesh = hex_plus_two_with_position();
    auto e1 = example_mesh.edge_tuple_between_v1_v2(3, 4, 0);
    auto e2 = example_mesh.edge_tuple_between_v1_v2(4, 0, 1);
    auto e3 = example_mesh.edge_tuple_between_v1_v2(4, 5, 2);
    auto e4 = example_mesh.edge_tuple_between_v1_v2(5, 1, 3);


    const TriMesh tri_mesh = static_cast<const TriMesh&>(example_mesh);

    TriMeshValenceEnergy valence_energy(tri_mesh);


    REQUIRE(valence_energy.energy_eval(e1) == 2);
    REQUIRE(valence_energy.energy_eval(e2) == 2);
    REQUIRE(valence_energy.energy_eval(e3) == 2);
    REQUIRE(valence_energy.energy_eval(e4) == 2);
}

TEST_CASE("amips2d")
{
    SECTION("equilateral triangle")
    {
        const DEBUG_TriMesh example_mesh = single_equilateral_triangle();
        auto e1 = example_mesh.edge_tuple_between_v1_v2(0, 1, 0);
        const TriMesh tri_mesh = static_cast<const TriMesh&>(example_mesh);

        AMIPS_2D amips2d(tri_mesh);

        REQUIRE(amips2d.energy_eval(e1) == 2.0);
    }
}