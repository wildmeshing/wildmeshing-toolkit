#include <wmtk/TriMeshEasy.h>
using namespace wmtk;
namespace {
class TriMeshEasyEdgeCollapseOperation : public wmtk::TriMeshOperationShim<
                                             TriMeshEasy,
                                             TriMeshEasyEdgeCollapseOperation,
                                             wmtk::TriMeshEdgeCollapseOperation>
{
public:
    ExecuteReturnData execute(TriMeshEasy& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeCollapseOperation::execute(m, t);
    }
    bool before(TriMeshEasy& m, const Tuple& t)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
            if (m.collapse_edge_before) {
                return m.collapse_edge_before(t);
            } else {
                return true;
            }
        }
        return false;
    }
    bool after(TriMeshEasy& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::after(m, ret_data)) {
            if (m.collapse_edge_after) {
                ret_data.success &= m.collapse_edge_after(ret_data);
            }
        }
        return ret_data;
    }
    bool invariants(TriMeshEasy& m, ExecuteReturnData& ret_data)
    {
        if (m.invariants) {
            ret_data.success &= m.invariants(ret_data);
        }
        return ret_data;
    }
};
class TriMeshEasySplitEdgeOperation : public wmtk::TriMeshOperationShim<
                                          TriMeshEasy,
                                          TriMeshEasySplitEdgeOperation,
                                          wmtk::TriMeshSplitEdgeOperation>
{
public:
    ExecuteReturnData execute(TriMeshEasy& m, const Tuple& t)
    {
        return wmtk::TriMeshSplitEdgeOperation::execute(m, t);
    }
    bool before(TriMeshEasy& m, const Tuple& t)
    {
        if (wmtk::TriMeshSplitEdgeOperation::before(m, t)) {
            if (m.split_edge_before) {
                return m.split_edge_before(t);
            } else {
                return true;
            }
        }
        return false;
    }
    bool after(TriMeshEasy& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSplitEdgeOperation::after(m, ret_data)) {
            if (m.split_edge_after) {
                ret_data.success &= m.split_edge_after(ret_data);
            }
        }
        return ret_data;
    }
    bool invariants(TriMeshEasy& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSplitEdgeOperation::invariants(m, ret_data)) {
            if (m.invariants) {
                ret_data.success &= m.invariants(ret_data);
            }
        }
        return ret_data;
    }
};

class TriMeshEasySwapEdgeOperation : public wmtk::TriMeshOperationShim<
                                         TriMeshEasy,
                                         TriMeshEasySwapEdgeOperation,
                                         wmtk::TriMeshSwapEdgeOperation>
{
public:
    ExecuteReturnData execute(TriMeshEasy& m, const Tuple& t)
    {
        return wmtk::TriMeshSwapEdgeOperation::execute(m, t);
    }
    bool before(TriMeshEasy& m, const Tuple& t)
    {
        if (wmtk::TriMeshSwapEdgeOperation::before(m, t)) {
            if (m.swap_edge_before) {
                return m.swap_edge_before(t);
            } else {
                return true;
            }
        }
        return false;
    }
    bool after(TriMeshEasy& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSwapEdgeOperation::after(m, ret_data)) {
            if (m.swap_edge_after) {
                ret_data.success &= m.swap_edge_after(ret_data);
            }
        }
        return ret_data;
    }
    bool invariants(TriMeshEasy& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSwapEdgeOperation::invariants(m, ret_data)) {
            if (m.invariants) {
                ret_data.success &= m.invariants(ret_data);
            }
        }
        return ret_data;
    }
};

class TriMeshEasySmoothVertexOperation : public wmtk::TriMeshOperationShim<
                                             TriMeshEasy,
                                             TriMeshEasySmoothVertexOperation,
                                             wmtk::TriMeshSmoothVertexOperation>
{
public:
    ExecuteReturnData execute(TriMeshEasy& m, const Tuple& t)
    {
        return wmtk::TriMeshSmoothVertexOperation::execute(m, t);
    }
    bool before(TriMeshEasy& m, const Tuple& t)
    {
        if (wmtk::TriMeshSmoothVertexOperation::before(m, t)) {
            if (m.vertex_smooth_before) {
                return m.vertex_smooth_before(t);
            } else {
                return true;
            }
        }
        return false;
    }
    bool after(TriMeshEasy& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSmoothVertexOperation::after(m, ret_data)) {
            if (m.vertex_smooth_after) {
                ret_data.success &= m.vertex_smooth_after(ret_data);
            }
        }
        return ret_data;
    }
    bool invariants(TriMeshEasy& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSmoothVertexOperation::invariants(m, ret_data)) {
            if (m.invariants) {
                ret_data.success &= m.invariants(ret_data);
            }
        }
        return ret_data;
    }
};


} // namespace

std::vector<std::shared_ptr<TriMeshOperation>> TriMeshEasy::getOperations() const
{
    std::vector<std::shared_ptr<TriMeshOperation>> e;
    e.emplace_back(std::make_shared<TriMeshEasyEdgeCollapseOperation>());
    e.emplace_back(std::make_shared<TriMeshEasySmoothVertexOperation>());
    e.emplace_back(std::make_shared<TriMeshEasySplitEdgeOperation>());
    e.emplace_back(std::make_shared<TriMeshEasySwapEdgeOperation>());
    return e;
}

