#include <catch2/catch_test_macros.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/Function.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
// #include <wmtk/function/PositionMapAMIPS2D.hpp>
#include <wmtk/function/simplex/EdgeValenceEnergy.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::function;
using namespace wmtk::tests;
using namespace wmtk::simplex;
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


    const TriMesh& tri_mesh = static_cast<const TriMesh&>(example_mesh);
    auto handle = tri_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);


    EdgeValenceEnergy valence_energy(tri_mesh, handle);


    REQUIRE(valence_energy.get_value(Simplex(PrimitiveType::Edge, e1)) == 2);
    REQUIRE(valence_energy.get_value(Simplex(PrimitiveType::Edge, e2)) == 2);
    REQUIRE(valence_energy.get_value(Simplex(PrimitiveType::Edge, e3)) == 2);
    REQUIRE(valence_energy.get_value(Simplex(PrimitiveType::Edge, e4)) == 2);
}

TEST_CASE("amips2d_values")
{
    SECTION("equilateral_triangle")
    {
        const DEBUG_TriMesh example_mesh = single_equilateral_triangle(2);

        auto uv_handle =
            example_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        auto e1 = example_mesh.edge_tuple_between_v1_v2(0, 1, 0);
        const TriMesh& tri_mesh = static_cast<const TriMesh&>(example_mesh);

        TriangleAMIPS amips2d(tri_mesh, uv_handle);

        CHECK(abs(amips2d.get_value(Simplex(PrimitiveType::Triangle, e1)) - 2.0) < 1e-6);
    }
    SECTION("random_triangle")
    {
        for (int i = 0; i < 50; i++) {
            const DEBUG_TriMesh example_mesh = single_2d_triangle_with_random_positions(123);

            auto uv_handle =
                example_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
            auto e1 = example_mesh.edge_tuple_between_v1_v2(0, 1, 0);
            const TriMesh& tri_mesh = static_cast<const TriMesh&>(example_mesh);

            TriangleAMIPS amips2d(tri_mesh, uv_handle);
            CHECK(amips2d.get_value(Simplex(PrimitiveType::Triangle, e1)) >= 2.);
        }
    }
}

// TEST_CASE("PositionMapAMIPS_values")
// {
//     SECTION("equilateral_triangle")
//     {
//         const DEBUG_TriMesh example_mesh = single_equilateral_triangle(2);

//         auto e1 = example_mesh.edge_tuple_between_v1_v2(0, 1, 0);
//         const TriMesh tri_mesh = static_cast<const TriMesh&>(example_mesh);
//         auto uv_handle =
//             example_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

//         PositionMapAMIPS2D amips3d(
//             tri_mesh,
//             uv_handle,
//             wmtk::image::SamplingAnalyticFunction::FunctionType::Linear,
//             0.0,
//             0.0,
//             1.0);

//         CHECK(amips3d.get_value(Simplex(PrimitiveType::Face, e1)) == 2.0);
//     }
//     SECTION("random_triangle")
//     {
//         for (int i = 0; i < 50; i++) {
//             const DEBUG_TriMesh example_mesh = single_2d_triangle_with_random_positions(123);

//             auto uv_handle =
//                 example_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
//             auto e1 = example_mesh.edge_tuple_between_v1_v2(0, 1, 0);
//             const TriMesh tri_mesh = static_cast<const TriMesh&>(example_mesh);

//             PositionMapAMIPS2D amips3d(
//                 tri_mesh,
//                 uv_handle,
//                 wmtk::image::SamplingAnalyticFunction::FunctionType::Linear,
//                 0.0,
//                 0.0,
//                 1.0);

//             CHECK(amips3d.get_value(Simplex(PrimitiveType::Face, e1)) >= 2.0);
//         }
//     }
// }
