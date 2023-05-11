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
        std::optional<int> curve_id = std::nullopt;
        std::vector<std::optional<int>>
            before_curve_ids; // record 1,2 curve_id and 4,5 curve_id if they exist
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
        // s2/2  1\s1                           s2/2'|1'\ s1
        //  / ---> \                             /->^｜v \   
        // /___0____\           ===>            /_0'_｜_6_\    
        // if size == 3, it's a boundary edge,
        // before_sibling_edges: nullopt, s1, s2
        // mirror data of ^ v in the middle needs to be nullified
        // so are the edge_attrs data of the middle edges
        // ___ 3 ____                           __7__ __3'_
        // \        /                           \    | <--/
        //  \ <--- /                             \4' | 5'/
        // s4\4  5/s5                           s4\ ^|v / s5
        //    \  /                                 \ | /
        //     \/                                   \ /
        // if size == 6, it's interior or seam edge
        // before_sibling_edges: 3, s1, s2, 0, s4, s5
        std::vector<std::optional<TriMesh::Tuple>> before_sibling_edges;
        // after_sibling_edges: 0', 1', 2', 6
        // after_sibling_edges: 0', 1', 2', 3', 4', 5', 6, 7
        std::vector<TriMesh::Tuple> after_sibling_edges;
    };
    tbb::enumerable_thread_specific<PairedOpCache> paired_op_cache;

public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
};
