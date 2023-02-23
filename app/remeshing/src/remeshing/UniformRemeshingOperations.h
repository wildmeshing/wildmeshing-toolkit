#pragma once
#include <wmtk/TriMeshOperation.h>
#include "UniformRemeshing.h"
namespace app::remeshing {
class UniformRemeshingEdgeCollapseOperation : public wmtk::TriMeshOperationShim<
                                                  UniformRemeshing,
                                                  UniformRemeshingEdgeCollapseOperation,
                                                  wmtk::TriMeshEdgeCollapseOperation>
{
public:
    ExecuteReturnData execute(const Tuple& t, UniformRemeshing& m)
    {
        return wmtk::TriMeshEdgeCollapseOperation::execute(t, m);
    }
    bool before_check(const Tuple& t, UniformRemeshing& m)
    {
        return wmtk::TriMeshEdgeCollapseOperation::before_check(t, m) && m.collapse_edge_before(t);
    }
    bool after_check(const ExecuteReturnData& ret_data, UniformRemeshing& m)
    {
        return wmtk::TriMeshEdgeCollapseOperation::after_check(ret_data, m) &&
               m.collapse_edge_after(ret_data.tuple);
    }
    bool invariants(const ExecuteReturnData& ret_data, UniformRemeshing& m)
    {
        return wmtk::TriMeshEdgeCollapseOperation::invariants(ret_data, m) &&
               m.invariants(ret_data.new_tris);
    }
};

class UniformRemeshingSplitEdgeOperation : public wmtk::TriMeshOperationShim<
                                               UniformRemeshing,
                                               UniformRemeshingSplitEdgeOperation,
                                               wmtk::TriMeshSplitEdgeOperation>
{
public:
    ExecuteReturnData execute(const Tuple& t, UniformRemeshing& m)
    {
        return wmtk::TriMeshSplitEdgeOperation::execute(t, m);
    }
    bool before_check(const Tuple& t, UniformRemeshing& m)
    {
        return wmtk::TriMeshSplitEdgeOperation::before_check(t, m) && m.split_edge_before(t);
    }
    bool after_check(const ExecuteReturnData& ret_data, UniformRemeshing& m)
    {
        return wmtk::TriMeshSplitEdgeOperation::after_check(ret_data, m) &&
               m.split_edge_after(ret_data.tuple);
    }
    bool invariants(const ExecuteReturnData& ret_data, UniformRemeshing& m)
    {
        return wmtk::TriMeshSplitEdgeOperation::invariants(ret_data, m) &&
               m.invariants(ret_data.new_tris);
    }
};

class UniformRemeshingSwapEdgeOperation : public wmtk::TriMeshOperationShim<
                                              UniformRemeshing,
                                              UniformRemeshingSwapEdgeOperation,
                                              wmtk::TriMeshSwapEdgeOperation>
{
public:
    ExecuteReturnData execute(const Tuple& t, UniformRemeshing& m)
    {
        return wmtk::TriMeshSwapEdgeOperation::execute(t, m);
    }
    bool before_check(const Tuple& t, UniformRemeshing& m)
    {
        return wmtk::TriMeshSwapEdgeOperation::before_check(t, m) && m.swap_edge_before(t);
    }
    bool after_check(const ExecuteReturnData& ret_data, UniformRemeshing& m)
    {
        return wmtk::TriMeshSwapEdgeOperation::after_check(ret_data, m) &&
               m.swap_edge_after(ret_data.tuple);
    }
    bool invariants(const ExecuteReturnData& ret_data, UniformRemeshing& m)
    {
        return wmtk::TriMeshSwapEdgeOperation::invariants(ret_data, m) &&
               m.invariants(ret_data.new_tris);
    }
};


class UniformRemeshingSmoothVertexOperation : public wmtk::TriMeshOperationShim<
                                                  UniformRemeshing,
                                                  UniformRemeshingSmoothVertexOperation,
                                                  wmtk::TriMeshSmoothVertexOperation>
{
public:
    ExecuteReturnData execute(const Tuple& t, UniformRemeshing& m)
    {
        return wmtk::TriMeshSmoothVertexOperation::execute(t, m);
    }
    bool before_check(const Tuple& t, UniformRemeshing& m)
    {
        return wmtk::TriMeshSmoothVertexOperation::before_check(t, m) && m.smooth_before(t);
    }
    bool after_check(const ExecuteReturnData& ret_data, UniformRemeshing& m)
    {
        return wmtk::TriMeshSmoothVertexOperation::after_check(ret_data, m) &&
               m.smooth_after(ret_data.tuple);
    }
    bool invariants(const ExecuteReturnData& ret_data, UniformRemeshing& m)
    {
        return wmtk::TriMeshSmoothVertexOperation::invariants(ret_data, m) &&
               m.invariants(ret_data.new_tris);
    }
};

template <typename Executor>
void addCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<UniformRemeshingEdgeCollapseOperation>());
    e.add_operation(std::make_shared<UniformRemeshingSplitEdgeOperation>());
    e.add_operation(std::make_shared<UniformRemeshingSwapEdgeOperation>());
    e.add_operation(std::make_shared<UniformRemeshingSmoothVertexOperation>());
}


} // namespace app::remeshing
