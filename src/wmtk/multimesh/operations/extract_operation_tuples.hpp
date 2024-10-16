#pragma once
#include <array>
#include <vector>
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/multimesh/operations/SplitReturnData.hpp>


namespace wmtk::multimesh::operations {
// class CollapseReturnData;
// class SplitReturnData;

using OperationTupleData = std::map<const Mesh*, std::vector<std::array<Tuple, 2>>>;
using OperationInOutData =
    std::map<const Mesh*, std::vector<std::tuple<simplex::NavigatableSimplex, wmtk::Tuple>>>;

OperationTupleData extract_operation_tuples(const CollapseReturnData& return_data);
OperationTupleData extract_operation_tuples(const SplitReturnData& return_data);

OperationInOutData extract_operation_in_out(const CollapseReturnData& return_data);
OperationInOutData extract_operation_in_out(const SplitReturnData& return_data);

} // namespace wmtk::multimesh::operations
