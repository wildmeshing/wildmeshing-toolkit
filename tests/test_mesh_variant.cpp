#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/utils/metaprogramming/as_mesh_variant.hpp>
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;


namespace {
struct DimFunctor
{
    // the dimension of the mesh we expect to see
    int operator()(const PointMesh&) const
    {
        spdlog::info("Mesh!");
        return 0;
    }
    int operator()(const EdgeMesh&) const
    {
        spdlog::info("TriMesh");
        return 2;
    }
    int operator()(const TriMesh&) const
    {
        spdlog::info("TriMesh");
        return 2;
    }
    int operator()(const TetMesh&) const
    {
        spdlog::info("TetMesh");
        return 3;
    }
    template <typename T>
        int operator()(std::reference_wrapper<T> ref) {
            return (*this)(ref.get());
        }

};

} // namespace


constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;

TEST_CASE("test_multi_mesh_print_visitor", "[multimesh][2D]")
{
    auto trimesh = wmtk::tests::single_triangle();
    auto trivar = utils::metaprogramming::as_mesh_variant(trimesh);

    //// just to prove that as_mesh_variant just reads from mesh internally
    Mesh& mesh = trimesh;
    auto trimvar = utils::metaprogramming::as_mesh_variant(mesh);

    auto tetmesh = wmtk::tests_3d::single_tet();
    auto tetvar = utils::metaprogramming::as_mesh_variant(tetmesh);
    CHECK(std::visit(DimFunctor{}, trivar) == 2);
    CHECK(std::visit(DimFunctor{}, trimvar) == 2);
    CHECK(std::visit(DimFunctor{}, tetvar) == 3);
}
