#pragma once
#include <array>
#include <vector>
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/multimesh/operations/SplitReturnData.hpp>


namespace wmtk::multimesh::operations {

std::map<const Mesh*, std::vector<std::array<Tuple, 2>>> extract_operation_tuples(
    const CollapseReturnData& return_data);
std::map<const Mesh*, std::vector<std::array<Tuple, 2>>> extract_operation_tuples(
    const SplitReturnData& return_data);

} // namespace wmtk::multimesh::operations
