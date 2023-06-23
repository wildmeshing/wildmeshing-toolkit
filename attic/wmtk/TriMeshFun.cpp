#include <wmtk/TriMeshFun.h>
#include <wmtk/operations/TriMeshOperationShim.hpp>
#include <wmtk/operations/TriMeshEdgeSwapOperation.h>
#include <wmtk/operations/TriMeshEdgeSplitOperation.h>
#include <wmtk/operations/TriMeshEdgeCollapseOperation.h>
#include <wmtk/operations/TriMeshVertexSmoothOperation.h>
using namespace wmtk;
namespace {
class TriMeshFunEdgeCollapseOperation : public wmtk::TriMeshOperationShim<
                                             TriMeshFun,
                                             TriMeshFunEdgeCollapseOperation,
                                             wmtk::TriMeshEdgeCollapseOperation>
{
public:
    bool execute(TriMeshFun& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeCollapseOperation::execute(m, t);
    }
    bool before(TriMeshFun& m, const Tuple& t)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
            if (m.collapse_edge_before) {
                return m.collapse_edge_before(t);
            }
        }
        return false;
    }
    bool after(TriMeshFun& m)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::after(m)) {
            if (m.collapse_edge_after) {
                return m.collapse_edge_after(get_return_tuple_opt().value());
            }
        }
        return false;
    }
};
class TriMeshFunEdgeSplitOperation : public wmtk::TriMeshOperationShim<
                                          TriMeshFun,
                                          TriMeshFunEdgeSplitOperation,
                                          wmtk::TriMeshEdgeSplitOperation>
{
public:
    bool execute(TriMeshFun& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeSplitOperation::execute(m, t);
    }
    bool before(TriMeshFun& m, const Tuple& t)
    {
        if (wmtk::TriMeshEdgeSplitOperation::before(m, t)) {
            if (m.split_edge_before) {
                return m.split_edge_before(t);
            }
        }
        return false;
    }
    bool after(TriMeshFun& m)
    {
        if (wmtk::TriMeshEdgeSplitOperation::after(m)) {
            if (m.split_edge_after) {
                return m.split_edge_after(get_return_tuple_opt().value());
            }
        }
        return false;
    }
};

class TriMeshFunEdgeSwapOperation : public wmtk::TriMeshOperationShim<
                                         TriMeshFun,
                                         TriMeshFunEdgeSwapOperation,
                                         wmtk::TriMeshEdgeSwapOperation>
{
public:
    bool execute(TriMeshFun& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeSwapOperation::execute(m, t);
    }
    bool before(TriMeshFun& m, const Tuple& t)
    {
        if (wmtk::TriMeshEdgeSwapOperation::before(m, t)) {
            if (m.swap_edge_before) {
                return m.swap_edge_before(t);
            }
        }
        return false;
    }
    bool after(TriMeshFun& m)
    {
        if (wmtk::TriMeshEdgeSwapOperation::after(m)) {
            if (m.swap_edge_after) {
                return m.swap_edge_after(get_return_tuple_opt().value());
            }
        }
        return false;
    }
};

class TriMeshFunVertexSmoothOperation : public wmtk::TriMeshOperationShim<
                                             TriMeshFun,
                                             TriMeshFunVertexSmoothOperation,
                                             wmtk::TriMeshVertexSmoothOperation>
{
public:
    bool execute(TriMeshFun& m, const Tuple& t)
    {
        return wmtk::TriMeshVertexSmoothOperation::execute(m, t);
    }
    bool before(TriMeshFun& m, const Tuple& t)
    {
        if (wmtk::TriMeshVertexSmoothOperation::before(m, t)) {
            if (m.vertex_smooth_before) {
                return m.vertex_smooth_before(t);
            }
        }
        return false;
    }
    bool after(TriMeshFun& m)
    {
        if (wmtk::TriMeshVertexSmoothOperation::after(m)) {
            if (m.vertex_smooth_after) {
                return m.vertex_smooth_after(get_return_tuple_opt().value());
            }
        }
        return false;
    }
};



} // namespace

bool TriMeshFun::invariants(const TriMeshOperation& op) {
    return fun_invariants(op.modified_triangles(*this));
}

std::map<std::string, std::shared_ptr<TriMeshOperation>> TriMeshFun::get_operations() const
{
    std::map<std::string, std::shared_ptr<TriMeshOperation>> r;
    auto add_operation = [&](auto&& op) { r[op->name()] = op; };
    add_operation(std::make_shared<TriMeshFunEdgeCollapseOperation>());
    add_operation(std::make_shared<TriMeshFunVertexSmoothOperation>());
    add_operation(std::make_shared<TriMeshFunEdgeSplitOperation>());
    add_operation(std::make_shared<TriMeshFunEdgeSwapOperation>());
    return r;
}

