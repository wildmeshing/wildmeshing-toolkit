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
    ExecuteReturnData execute(UniformRemeshing& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeCollapseOperation::execute(m, t);
    }
    bool before(UniformRemeshing& m, const Tuple& t)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
            return  m.collapse_edge_before(t);
        }
        return false;
    }
    bool after(UniformRemeshing& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::after(m, ret_data)) {
            ret_data.success &= m.collapse_edge_after(ret_data.tuple);
        }
        return ret_data;
    }
    bool invariants(UniformRemeshing& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::invariants(m, ret_data)) {
            ret_data.success &= m.invariants(ret_data.new_tris);
        }
        return ret_data;
    }
};

class UniformRemeshingSplitEdgeOperation : public wmtk::TriMeshOperationShim<
                                               UniformRemeshing,
                                               UniformRemeshingSplitEdgeOperation,
                                               wmtk::TriMeshSplitEdgeOperation>
{
public:
    ExecuteReturnData execute(UniformRemeshing& m, const Tuple& t)
    {
        return wmtk::TriMeshSplitEdgeOperation::execute(m, t);
    }
    bool before(UniformRemeshing& m, const Tuple& t)
    {
        if (wmtk::TriMeshSplitEdgeOperation::before(m, t)) {
            return  m.split_edge_before(t);
        }
        return false;
    }
    bool after(UniformRemeshing& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSplitEdgeOperation::after(m, ret_data)) {
            ret_data.success |= m.split_edge_after(ret_data.tuple);
        }
        return ret_data;
    }
    bool invariants(UniformRemeshing& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSplitEdgeOperation::invariants(m, ret_data)) {
            ret_data.success |= m.invariants(ret_data.new_tris);
        }
        return ret_data;
    }
};

class UniformRemeshingSwapEdgeOperation : public wmtk::TriMeshOperationShim<
                                              UniformRemeshing,
                                              UniformRemeshingSwapEdgeOperation,
                                              wmtk::TriMeshSwapEdgeOperation>
{
public:
    ExecuteReturnData execute(UniformRemeshing& m, const Tuple& t)
    {
        return wmtk::TriMeshSwapEdgeOperation::execute(m, t);
    }
    bool before(UniformRemeshing& m, const Tuple& t)
    {
        if (wmtk::TriMeshSwapEdgeOperation::before(m, t)) {
            return  m.swap_edge_before(t);
        }
        return false;
    }
    bool after(UniformRemeshing& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSwapEdgeOperation::after(m, ret_data)) {
            ret_data.success |= m.swap_edge_after(ret_data.tuple);
        }
        return ret_data;
    }
    bool invariants(UniformRemeshing& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSwapEdgeOperation::invariants(m, ret_data)) {
            ret_data.success |= m.invariants(ret_data.new_tris);
        }
        return ret_data;
    }
};


class UniformRemeshingSmoothVertexOperation : public wmtk::TriMeshOperationShim<
                                                  UniformRemeshing,
                                                  UniformRemeshingSmoothVertexOperation,
                                                  wmtk::TriMeshSmoothVertexOperation>
{
public:
    ExecuteReturnData execute(UniformRemeshing& m, const Tuple& t)
    {
        return wmtk::TriMeshSmoothVertexOperation::execute(m, t);
    }
    bool before(UniformRemeshing& m, const Tuple& t)
    {
        if (wmtk::TriMeshSmoothVertexOperation::before(m, t)) {
            return  m.smooth_before(t);
        }
        return false;
    }
    bool after(UniformRemeshing& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSmoothVertexOperation::after(m, ret_data)) {
            ret_data.success |= m.smooth_after(ret_data.tuple);
        }
        return ret_data;
    }
    bool invariants(UniformRemeshing& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSmoothVertexOperation::invariants(m, ret_data)) {
            ret_data.success |= m.invariants(ret_data.new_tris);
        }
        return ret_data;
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
