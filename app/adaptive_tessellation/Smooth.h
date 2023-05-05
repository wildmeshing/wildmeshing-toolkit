
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/ScalarUtils.h>
#include <array>
#include <limits>
#include <optional>
#include <typeinfo>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include "AdaptiveTessellation.h"
#include "wmtk/ExecutionScheduler.hpp"
using namespace adaptive_tessellation;
using namespace wmtk;

class AdaptiveTessellationSmoothVertexOperation : public wmtk::TriMeshOperationShim<
                                                      AdaptiveTessellation,
                                                      AdaptiveTessellationSmoothVertexOperation,
                                                      wmtk::TriMeshSmoothVertexOperation>
{
public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
    bool invariants(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
};

class AdaptiveTessellationPairedSmoothVertexOperation
    : public wmtk::TriMeshOperationShim<
          AdaptiveTessellation,
          AdaptiveTessellationPairedSmoothVertexOperation,
          wmtk::TriMeshSmoothVertexOperation>
{
public:
    AdaptiveTessellationCollapseEdgeOperation collapse_edge;
    AdaptiveTessellationCollapseEdgeOperation collapse_mirror_edge;
    std::optional<Tuple> mirror_edge_tuple;

public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
    bool invariants(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
};