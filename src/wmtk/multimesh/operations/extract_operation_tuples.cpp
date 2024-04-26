#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/multimesh/operations/SplitReturnData.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_split.hpp>
#include <wmtk/operations/utils/ExtractOperationTuplesFunctor.hpp>


#include "extract_operation_tuples.hpp"

namespace wmtk::multimesh::operations {


std::map<const Mesh*, std::vector<std::array<Tuple, 2>>> extract_operation_tuples(
    const wmtk::operations::utils::CollapseReturnData& return_data)
{
    std::map<const Mesh*, std::vector<std::array<Tuple, 2>>> ret;

    for (const auto& [key, value_var] : return_data) {
        const auto [mesh_ptr, input_simplex] = key;
        auto tups = std::visit(wmtk::operations::utils::ExtractOperationTuplesFunctor{}, value_var);
        assert(tups[0] == input_simplex.tuple());
        ret[mesh_ptr].emplace_back(tups);
    }

    return ret;
}
std::map<const Mesh*, std::vector<std::array<Tuple, 2>>> extract_operation_tuples(
    const wmtk::operations::utils::SplitReturnData& return_data)
{
    std::map<const Mesh*, std::vector<std::array<Tuple, 2>>> ret;

    for (const auto& [key, value_var] : return_data) {
        const auto [mesh_ptr, input_simplex] = key;
        auto tups = std::visit(wmtk::operations::utils::ExtractOperationTuplesFunctor{}, value_var);
        assert(tups[0] == input_simplex.tuple());
        ret[mesh_ptr].emplace_back(tups);
    }

    return ret;
}
} // namespace wmtk::multimesh::operations
