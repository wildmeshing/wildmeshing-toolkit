#pragma once
#include <wmtk/TriMeshOperation.h>
#include <wmtk/operations/TriMeshOperationShim.hpp>
#include <wmtk/operations/TriMeshEdgeCollapseOperation.h>
#include <wmtk/operations/TriMeshEdgeSplitOperation.h>
#include <wmtk/operations/TriMeshEdgeSwapOperation.h>
#include <wmtk/operations/TriMeshVertexSmoothOperation.h>
#include "UniformRemeshing.h"
namespace app::remeshing {
class UniformRemeshingEdgeCollapseOperation : public wmtk::TriMeshOperationShim<
                                                  UniformRemeshing,
                                                  UniformRemeshingEdgeCollapseOperation,
                                                  wmtk::TriMeshEdgeCollapseOperation>
{
public:
    bool execute(UniformRemeshing& m, const Tuple& t)
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
    bool after(UniformRemeshing& m)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::after(m)) {
            return m.collapse_edge_after(get_return_tuple_opt().value());
        }
        return false;
    }
};

class UniformRemeshingEdgeSplitOperation : public wmtk::TriMeshOperationShim<
                                               UniformRemeshing,
                                               UniformRemeshingEdgeSplitOperation,
                                               wmtk::TriMeshEdgeSplitOperation>
{
public:
    bool execute(UniformRemeshing& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeSplitOperation::execute(m, t);
    }
    bool before(UniformRemeshing& m, const Tuple& t)
    {
        if (wmtk::TriMeshEdgeSplitOperation::before(m, t)) {
            return  m.split_edge_before(t);
        }
        return false;
    }
    bool after(UniformRemeshing& m)
    {
        if (wmtk::TriMeshEdgeSplitOperation::after(m)) {
            return m.split_edge_after(get_return_tuple_opt().value());
        }
        return false;
    }
};

class UniformRemeshingEdgeSwapOperation : public wmtk::TriMeshOperationShim<
                                              UniformRemeshing,
                                              UniformRemeshingEdgeSwapOperation,
                                              wmtk::TriMeshEdgeSwapOperation>
{
public:
    bool execute(UniformRemeshing& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeSwapOperation::execute(m, t);
    }
    bool before(UniformRemeshing& m, const Tuple& t)
    {
        if (wmtk::TriMeshEdgeSwapOperation::before(m, t)) {
            return  m.swap_edge_before(t);
        }
        return false;
    }
    bool after(UniformRemeshing& m)
    {
        if (wmtk::TriMeshEdgeSwapOperation::after(m)) {
            return m.swap_edge_after(get_return_tuple_opt().value());
        }
        return false;
    }
};


class UniformRemeshingVertexSmoothOperation : public wmtk::TriMeshOperationShim<
                                                  UniformRemeshing,
                                                  UniformRemeshingVertexSmoothOperation,
                                                  wmtk::TriMeshVertexSmoothOperation>
{
public:
    bool execute(UniformRemeshing& m, const Tuple& t)
    {
        return wmtk::TriMeshVertexSmoothOperation::execute(m, t);
    }
    bool before(UniformRemeshing& m, const Tuple& t)
    {
        if (wmtk::TriMeshVertexSmoothOperation::before(m, t)) {
            return  m.smooth_before(t);
        }
        return false;
    }
    bool after(UniformRemeshing& m)
    {
        if (wmtk::TriMeshVertexSmoothOperation::after(m)) {
            return m.smooth_after(get_return_tuple_opt().value());
        }
        return false;
    }
};




} // namespace app::remeshing
