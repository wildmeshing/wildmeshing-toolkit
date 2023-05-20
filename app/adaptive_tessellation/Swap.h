#pragma once
#include <wmtk/TriMeshOperation.h>
#include "AdaptiveTessellation.h"

/// swap:       accuracy pass: done to improve accuracy of the mesh
///             quality pass: done to improve quality (valence for each vertex)
/// priority:   accuracy pass: summed one-ring quadric error of 4 vertices, priority =  before - after
///             quality pass: valence diffference to 6 for each vertex, priority =  before - after
/// scheduling; descending order of priority
/// stop cond:  if priority <0 (swap does not improve accruacy/quality), stop
//              quality pass stop when swap for quality exceeds the quality safeguard threshold
/// after:      update the quadric of the new faces. and after error is measured by the sum of the one-ring quadric error of 4 vertices


namespace adaptive_tessellation {
class AdaptiveTessellationSwapEdgeOperation : public wmtk::TriMeshOperationShim<
                                                  AdaptiveTessellation,
                                                  AdaptiveTessellationSwapEdgeOperation,
                                                  wmtk::TriMeshSwapEdgeOperation>
{
public:
    AdaptiveTessellationSwapEdgeOperation();
    ~AdaptiveTessellationSwapEdgeOperation();
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data);
    //        _____s3_____                _____s3_____
    //       / \    3    /               /     3' _ / /
    //      /    \     4/s4          s1 /1'  ___ / 4'/s4
    //   s1/ 1   0 \   /               / __/        /
    //    /____2_____\/               //_____2'____/
    //         s2                            s2
    // maps from an edge represented by two vids to the mirror edge's tuple, and boundary curve_id
    // only seam edge has mirror tuple and boundary edge has curve_id (thus they are std::optional)
    tbb::enumerable_thread_specific<
        std::map<std::array<size_t, 2>, std::pair<std::optional<Tuple>, int>>>
        vid_edge_to_mirror_edge;
    std::vector<Tuple> modified_triangles(const TriMesh& m) const override;
};
} // namespace adaptive_tessellation
