#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/TriMeshOperation.h>


namespace wmtk {


class TriMeshEasy : public TriMesh
{
public:
    using ExecuteReturnValue = TriMeshOperation::ExecuteReturnData;
    std::vector<std::shared_ptr<TriMeshOperation>> getOperations() const;


    std::function<bool(const Tuple&)> collapse_edge_before;
    std::function<bool(ExecuteReturnValue&)> collapse_edge_after;
    std::function<bool(const Tuple&)> split_edge_before;
    std::function<bool(ExecuteReturnValue&)> split_edge_after;
    std::function<bool(const Tuple&)> swap_edge_before;
    std::function<bool(ExecuteReturnValue&)> swap_edge_after;
    std::function<bool(const Tuple&)> vertex_smooth_before;
    std::function<bool(ExecuteReturnValue&)> vertex_smooth_after;
    std::function<bool(ExecuteReturnValue&)> invariants;
};
} // namespace wmtk
