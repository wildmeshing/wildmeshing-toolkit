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


// CollapseEdgeOperation stores some details on the collapse that it occurred
// CollapseEdgePairOperation
//


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
        size_t v1 = 0; // first vertex index of the edge being collapsed
        size_t v2 = 0; // second vertex index of hte edge being collapsed

        double length3d = 0;

        // Given an input triangle with tuple X
        //               v_top
        //   ------------o-------------
        //   |\          /\v         /|
        //   | \        /  \        / |
        //   |  \      /    \      /  |
        //   |   \    /  X   \    /   |
        //   |    \  /        \  /    |
        //   |     \/          \/     |
        //   ------X-----X-------------
        //   |     /\          /\     |
        //   |    /  \        /  \    |
        //   |   /    \      /    \   |
        //   |  /      \    /      \  |
        //   | /        \  /        \ |
        //   |/          \/          \|
        //   ----------- -o------------
        //              v_bot
        //
        // it is transformed to
        //   ---------------
        //   |\     |     /|
        //   | \    |    / |
        //   |  \   |   /  |
        //   |   \  |  /   |
        //   |    \ | /    |
        //   |     \|/     |
        //   ---------------
        //   |     /|\     |
        //   |    / | \    |
        //   |   /  |  \   |
        //   |  /   |   \  |
        //   | /    |    \ |
        //   |/     |     \|
        //   ---------------
        std::optional<size_t> v_top = 0; // remaining vertex of the input triangle being collapsed
        std::optional<size_t> v_bot = 0; // remaining vertex of the other triangle being collapsed


        // pairs of vids for edges where a vertex changes in the collapse
        std::unordered_map<size_t, SeamData> new_vertex_seam_data;
    };
    tbb::enumerable_thread_specific<OpCache> m_op_cache;

    static bool check_seamed_link_condition(AdaptiveTessellation& m, const Tuple& t);

    // computes the
    static LinksOfVertex seamed_links_of_vertex(AdaptiveTessellation& m, const Tuple& vertex);
    static std::vector<size_t> seamed_edge_link_of_edge(AdaptiveTessellation& m, const Tuple& edge);

public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
    bool after(AdaptiveTessellation& m);
};

class AdaptiveTessellationPairedCollapseEdgeOperation
    : public wmtk::TriMeshOperationShim<
          AdaptiveTessellation,
          AdaptiveTessellationPairedCollapseEdgeOperation,
          wmtk::TriMeshOperation>
{
public:
    AdaptiveTessellationCollapseEdgeOperation collapse_edge;
    AdaptiveTessellationCollapseEdgeOperation collapse_mirror_edge;
    std::string name() const override { return collapse_edge.name(); }
    using SeamData = AdaptiveTessellationCollapseEdgeOperation::SeamData;

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
        // when the input is a seam
        std::optional<Tuple> mirror_edge_tuple_opt;

        // pairs of vids for edges where no vertex changes in the collapse
        // TODO: add a hash and convert to unordered_map
        std::map<std::array<size_t, 2>, SeamData> seam_data;
    };
    mutable tbb::enumerable_thread_specific<OpCache> m_op_cache;

    // std::vector<Tuple> modified_tuples(const TriMesh& m);

public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
    bool after(AdaptiveTessellation& m);
    operator bool() const;
    std::vector<Tuple> modified_tuples(const AdaptiveTessellation& m) const;

    void mark_failed() override;

private:
    // stores the input edge's mirror for future use (if applicable)
    void set_input_edge_mirror(const AdaptiveTessellation& m, const Tuple& t);
    bool input_edge_is_mirror() const;

    std::optional<Tuple> get_mirror_edge_tuple_opt() const;


    // stores information about curves and mirrors to be rebuilt later
    void store_boundary_data(const AdaptiveTessellation& m, const Tuple& t);
    // populates curves and mirrors in the updated mesh
    void rebuild_boundary_data(AdaptiveTessellation& m);
};
} // namespace adaptive_tessellation
