#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;


namespace {
struct PrintTypeSizeFunctor
{
    void operator()(const Mesh&, const Simplex&) const { spdlog::warn("Unimplemented!"); }
    void operator()(const TriMesh& m, const Simplex&) const
    {
        spdlog::info(
            "TriMesh: {} (path: {})",
            m.capacity(PrimitiveType::Face),
            m.absolute_multi_mesh_id());
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
    void operator()(const Mesh&, const std::string& a, const Mesh&, const std::string& b) const
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

namespace {
// Declare a base type that has some sort of ID member and a way of identifying
// an appropriate derived type
struct Input
{
    int type = -1;
    int id;
};

// Some example derived types
struct A : public Input
{
    A(int id)
        : Input{0, id}
    {}
};

struct B : public Input
{
    B(int id)
        : Input{1, id}
    {}
};

struct C : public Input
{
    C(int id)
        : Input{2, id}
    {}
};


struct TestFunctor
{
    template <typename T>
    auto operator()(T& input) const
    {
        using TT = std::unwrap_ref_decay_t<T>;
        return std::tuple<TT, int>(input, input.id);
    };
};

struct TestFunctor2Args
{
    template <typename T>
    auto operator()(T& input, int data) const
    {
        using TT = std::unwrap_ref_decay_t<T>;
        return std::tuple<TT, int>(input, input.id * data);
    };
};
} // namespace


TEST_CASE("test_multi_mesh_basic_visitor", "[multimesh][2D]")
{
    A a(0);
    B b(2);
    C c(4);

    // test calling the functor once
    {
        auto [ap, i] = TestFunctor{}(a);
        spdlog::info("{},{} = {}", ap.type, ap.id, i);
    }

    // create a mono arg
    Runner r(TestFunctor{});

    r.run(a);
    r.run(b);
    r.run(c);

    {
        auto [ap, i] = r.return_data.get(a);
        spdlog::info("{},{} = {}", ap.type, ap.id, i);
    }

    {
        auto [ap, i] = r.return_data.get(b);
        spdlog::info("{},{} = {}", ap.type, ap.id, i);
    }
    {
        auto [ap, i] = r.return_data.get(c);
        spdlog::info("{},{} = {}", ap.type, ap.id, i);
    }

    // try using 2 args
    Runner r2(TestFunctor2Args{}, std::tuple<int>{});
    r2.run(a, 3);
    r2.run(b, 5);
    r2.run(c, 7);

    {
        auto [ap, i] = r2.return_data.get(a, 3);
        spdlog::info("{},{} = {}", ap.type, ap.id, i);
    }

    {
        auto [ap, i] = r2.return_data.get(b, 5);
        spdlog::info("{},{} = {}", ap.type, ap.id, i);
    }
    {
        auto [ap, i] = r2.return_data.get(c, 7);
        spdlog::info("{},{} = {}", ap.type, ap.id, i);
    }
}

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


    multimesh::MultiMeshVisitor print_type_visitor(PrintTypeSizeFunctor{});

    auto tups = parent.get_all(PrimitiveType::Face);
    for (const auto& t : tups) {
        print_type_visitor.execute_from_root(parent, Simplex(PF, t));
    }

    spdlog::warn("edge visitor!");
    multimesh::MultiMeshVisitor print_edge_visitor(
        GetTypeSizeFunctorWithReturn{},
        PrintEdgeReturnsFunctor{});

    for (const auto& t : tups) {
        print_edge_visitor.execute_from_root(parent, Simplex(PF, t));
    }
}
