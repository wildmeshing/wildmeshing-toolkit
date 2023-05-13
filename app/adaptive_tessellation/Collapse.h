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

namespace adaptive_tessellation {
class AdaptiveTessellationCollapseEdgeOperation : public wmtk::TriMeshOperationShim<
                                                      AdaptiveTessellation,
                                                      AdaptiveTessellationCollapseEdgeOperation,
                                                      wmtk::TriMeshEdgeCollapseOperation>
{
public:
    struct SeamData
    {
        std::optional<Tuple> mirror_edge_tuple;
        size_t curve_id;
    };
    struct OpCache
    {
        size_t v1; // first vertex index of the edge being collapsed
        size_t v2; // second vertex index of hte edge being collapsed

        double length3d;

        size_t v_top; // remaining vertex of the input triangle being collapsed
        size_t v_bot; // remaining vertex of the other triangle being collapsed

        // pairs of vids for edges where no vertex changes in the collapse
        std::map<std::array<size_t, 2>, SeamData> seam_data;

        // pairs of vids for edges where a vertex changes in the collapse
        std::unordered_map<size_t, SeamData> new_vertex_seam_data;
    };
    tbb::enumerable_thread_specific<OpCache> m_op_cache;

    static bool check_seamed_link_condition(AdaptiveTessellation& m, const Tuple& t);

    // computes the
    static LinksOfVertex seamed_links_of_vertex(AdaptiveTessellation& m, const Tuple& vertex);
    static std::vector<size_t> seamed_edge_link_of_edge(
        AdaptiveTessellation& m,
        const Tuple& edge);

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

    struct OpCache
    {
        std::optional<Tuple> mirror_edge_tuple_opt;
    };
    tbb::enumerable_thread_specific<OpCache> m_op_cache;

    // std::vector<Tuple> modified_tuples(const TriMesh& m);
    operator bool();

public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
};
} // namespace adaptive_tessellation
