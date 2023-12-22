#include <catch2/catch_test_macros.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/multimesh/MultiMeshSimplexEventVisitor.hpp>
#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;


namespace {
struct PrintTypeSizeFunctor
{
    int operator()(const Mesh&, const Simplex&) const
    {
        spdlog::warn("Unimplemented!");
        return 0;
    }
    long operator()(const TriMesh& m, const Simplex&) const
    {
        spdlog::info(
            "TriMesh: {} (path: {})",
            m.capacity(PrimitiveType::Face),
            m.absolute_multi_mesh_id());
        return 0.0;
    }
};

struct GetTypeSizeFunctorWithReturn
{
    std::string operator()(const Mesh&, const Simplex&) const { return "Unimplemented!"; }
    std::string operator()(const TriMesh& m, const Simplex&) const
    {
        spdlog::info("Do stuff!");
        return fmt::format(
            "[TriMesh: {} (path: {})]",
            m.capacity(PrimitiveType::Face),
            m.absolute_multi_mesh_id());
    }
};

struct PrintEdgeReturnsFunctor
{
    void operator()(
        const Mesh&,
        const Simplex&,
        const std::string& a,
        const Mesh&,
        const Simplex&,
        const std::string& b) const
    {
        spdlog::error("[{}] => [{}]", a, b);
    }
};
} // namespace


using TM = TriMesh;
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<Accessor<long>&>()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;


TEST_CASE("test_multi_mesh_print_visitor", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::shared_ptr<DEBUG_TriMesh> child2_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());

    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;
    auto& child2 = *child2_ptr;

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {0});
    auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});
    auto child2_map = multimesh::same_simplex_dimension_surjection(parent, child2, {0, 1, 2});

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);
    parent.register_child_mesh(child2_ptr, child2_map);


    multimesh::MultiMeshSimplexVisitor print_type_visitor(PrintTypeSizeFunctor{});

    auto tups = parent.get_all(PrimitiveType::Face);
    for (const auto& t : tups) {
        print_type_visitor.execute_from_root(static_cast<TriMesh&>(parent), Simplex(PF, t));
    }

    spdlog::warn("edge visitor!");
    multimesh::MultiMeshSimplexVisitor print_edge_visitor(GetTypeSizeFunctorWithReturn{});


    for (const auto& t : tups) {
        print_edge_visitor.execute_from_root(parent, Simplex(PF, t));
        multimesh::MultiMeshSimplexEventVisitor print_edge_event_visitor(print_edge_visitor);
        print_edge_event_visitor.run_on_edges(PrintEdgeReturnsFunctor{});
    }
}
