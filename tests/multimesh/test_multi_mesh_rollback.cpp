#include <catch2/catch_test_macros.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/invariants/internal/ConstantInvariant.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include "../tools/DEBUG_EdgeMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::simplex;

using TM = TriMesh;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;


TEST_CASE("split_all_fail_multimesh", "[operations][split]")
{
    // creates N triangles surrounding a single interior vertex 0
    int number = 20;
    auto d = disk(number);
    auto i = disk(number);
    auto map = multimesh::same_simplex_dimension_bijection(*d, *i);

    d->register_child_mesh(i, map);
    // auto parent_ptr = disk_to_individual_multimesh(number);
    // auto child_ptr = parent_ptr->get_multi_mesh_mesh({0}).shared_from_this();
    auto parent_ptr = d;
    auto child_ptr = i;

    auto fail_after =
        std::make_shared<wmtk::invariants::internal::ConstantInvariant>(*child_ptr, true, false);


    operations::EdgeSplit split_op(*parent_ptr);

    split_op.add_invariant(fail_after);

    for (int j = 0; j < 5; ++j) {
        for (const auto& tup : parent_ptr->get_all(wmtk::PrimitiveType::Edge)) {
            CHECK(split_op(simplex::Simplex::edge(*parent_ptr, tup)).empty());
        }
    }
}
