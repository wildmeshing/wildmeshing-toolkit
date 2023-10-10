#pragma once
#include <catch2/catch_test_macros.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/AMIPS.hpp>
#include <wmtk/function/TriMeshValenceFunction.hpp>
#include <wmtk/function/utils/DofsToPosition.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::function;
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

    TriMeshValenceFunction valence_energy(tri_mesh);


    REQUIRE(valence_energy.get_value(e1) == 2);
    REQUIRE(valence_energy.get_value(e2) == 2);
    REQUIRE(valence_energy.get_value(e3) == 2);
    REQUIRE(valence_energy.get_value(e4) == 2);
}

TEST_CASE("amips2d")
{
    SECTION("equilateral_triangle")
    {
        const DEBUG_TriMesh example_mesh = single_equilateral_triangle(2);

        auto uv_handle =
            example_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
        auto e1 = example_mesh.edge_tuple_between_v1_v2(0, 1, 0);
        const TriMesh tri_mesh = static_cast<const TriMesh&>(example_mesh);

        AMIPS_2D amips2d(tri_mesh, uv_handle);

        REQUIRE(amips2d.get_value(e1) == 2.0);
    }
    SECTION("random_triangle")
    {
        for (int i = 0; i < 1; i++) {
            const DEBUG_TriMesh example_mesh = single_2d_triangle_with_position();

            auto uv_handle =
                example_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
            auto e1 = example_mesh.edge_tuple_between_v1_v2(0, 1, 0);
            const TriMesh tri_mesh = static_cast<const TriMesh&>(example_mesh);

            AMIPS_2D amips2d(tri_mesh, uv_handle);
            if (amips2d.get_value(e1) < 2.) {
                wmtk::logger().critical("wrong value");
                REQUIRE((amips2d.get_value(e1) > 2. || amips2d.get_value(e1) == 2.));
            }
        }
    }
}

TEST_CASE("amips3d")
{
    SECTION("equilateral_triangle")
    {
        const DEBUG_TriMesh example_mesh = single_equilateral_triangle(2);

        auto e1 = example_mesh.edge_tuple_between_v1_v2(0, 1, 0);
        const TriMesh tri_mesh = static_cast<const TriMesh&>(example_mesh);
        auto uv_handle =
            example_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);

        AMIPS_3DEmbedded amips3d(
            tri_mesh,
            uv_handle,
            wmtk::image::SamplingAnalyticFunction::FunctionType::Linear,
            0.0,
            0.0,
            1.0);

        REQUIRE(amips3d.get_value(e1) == 2.0);
    }
    SECTION("random_triangle")
    {
        for (int i = 0; i < 50; i++) {
            const DEBUG_TriMesh example_mesh = single_triangle_with_position(2);

            auto uv_handle =
                example_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
            auto e1 = example_mesh.edge_tuple_between_v1_v2(0, 1, 0);
            const TriMesh tri_mesh = static_cast<const TriMesh&>(example_mesh);

            AMIPS_3DEmbedded amips3d(
                tri_mesh,
                uv_handle,
                wmtk::image::SamplingAnalyticFunction::FunctionType::Linear,
                0.0,
                0.0,
                1.0);

            REQUIRE((amips3d.get_value(e1) > 2. || amips3d.get_value(e1) == 2.));
        }
    }
}