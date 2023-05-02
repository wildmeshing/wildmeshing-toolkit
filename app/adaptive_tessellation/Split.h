#pragma once
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

class AdaptiveTessellationSplitEdgeOperation : public wmtk::TriMeshOperationShim<
                                                   AdaptiveTessellation,
                                                   AdaptiveTessellationSplitEdgeOperation,
                                                   wmtk::TriMeshSplitEdgeOperation>
{
public:
    TriMesh::Tuple return_edge_tuple;
    struct OpCache
    {
        size_t v1;
        size_t v2;
    };
    tbb::enumerable_thread_specific<OpCache> op_cache;

public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
};

class AdaptiveTessellationPairedSplitEdgeOperation
    : public wmtk::TriMeshOperationShim<
          AdaptiveTessellation,
          AdaptiveTessellationPairedSplitEdgeOperation,
          wmtk::TriMeshSplitEdgeOperation>
{
public:
    AdaptiveTessellationSplitEdgeOperation split_edge;
    AdaptiveTessellationSplitEdgeOperation mirror_split_edge;
    std::optional<Tuple> mirror_edge_tuple; // same orientation as the original edge
    struct PairedOpCache
    {
        // split edge is edge 0
        //     /\                                   / \ 
        //    /  \                                 / ｜\ 
        // s2/2  1\s1                           s2/ ^｜9\ s1
        //  / ---> \                             /->8｜v \   
        // /___0____\           ===>            /_0'_｜_6_\ 
        // if size == 3, it's a boundary edge,
        // sibling edges: nullopt, s1, s2
        // ___ 3 ____                           __7__ __3'_
        // \        /                           \    | <--/
        //  \ <--- /                             \ 10|11 /
        // s4\4  5/s5                           s4\ ^|v / s5
        //    \  /                                 \ | /
        //     \/                                   \ /
        // if size == 6, it's interior or seam edge
        // sibling edges: 3, s1, s2, 0, s4, s5
        std::vector<std::optional<TriMesh::Tuple>> sibling_edges;
    };
    tbb::enumerable_thread_specific<PairedOpCache> paired_op_cache;

public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
};
