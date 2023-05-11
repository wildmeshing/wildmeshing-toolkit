#pragma once
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/ScalarUtils.h>
#include <array>
#include <limits>
#include <optional>
#include <type_traits>
#include <typeinfo>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include "AdaptiveTessellation.h"
#include "wmtk/ExecutionScheduler.hpp"
using namespace adaptive_tessellation;
using namespace wmtk;

class AdaptiveTessellationCollapseEdgeOperation : public wmtk::TriMeshOperationShim<
                                                      AdaptiveTessellation,
                                                      AdaptiveTessellationCollapseEdgeOperation,
                                                      wmtk::TriMeshEdgeCollapseOperation>
{
public:
    TriMesh::Tuple return_edge_tuple;
    struct OpCache
    {
        size_t v1;
        size_t v2;
        double length3d;
    };
    tbb::enumerable_thread_specific<OpCache> op_cache;

public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
};

class AdaptiveTessellationPairedCollapseEdgeOperation
    : public wmtk::TriMeshOperationShim<
          AdaptiveTessellation,
          AdaptiveTessellationPairedCollapseEdgeOperation,
          wmtk::TriMeshEdgeCollapseOperation>
{
public:
    AdaptiveTessellationCollapseEdgeOperation collapse_edge;
    AdaptiveTessellationCollapseEdgeOperation collapse_mirror_edge;
    std::optional<Tuple> mirror_edge_tuple;

    //        / | \                         |\ 
    //       /  |  \                        | \ 
    //    s2/2 1|s1 \                    \s2|s1\ 
    // \ | /-> ^｜v  \                    \^|v  \ 
    // _\|/_0 __｜____\                  __\|____\ 
    //          |\                          |\ 
    //update the one ring edges' mirror edge data
    // __ ___3__|/____                   __ |/____
    //  /|\  <--|    /                    / |    /
    //   s4\4  5|s5 /                    /s4|s5 /
    //      \  ^|v /                        |v /
    //       \  | /                         | /
    //        \ |/                          |/


public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
};