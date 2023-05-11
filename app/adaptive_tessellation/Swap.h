#pragma once
#include <wmtk/TriMeshOperation.h>
#include "AdaptiveTessellation.h"

namespace adaptive_tessellation
{
    class AdaptiveTessellationSwapEdgeOperation : public wmtk::TriMeshOperationShim<
                                                      AdaptiveTessellation,
                                                      AdaptiveTessellationSwapEdgeOperation,
                                                      wmtk::TriMeshSwapEdgeOperation>
    {
    public:
        AdaptiveTessellationSwapEdgeOperation ();
        ~AdaptiveTessellationSwapEdgeOperation ();
        ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
        bool before(AdaptiveTessellation& m, const Tuple& t);
        bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);


        // maps from an edge represented by two vids to the mirror edge's tuple
        tbb::enumerable_thread_specific<std::map<std::array<size_t, 2>, Tuple>>
            vid_edge_to_mirror_edge;
    };
}
