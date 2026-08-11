#pragma once

#include <wmtk/OptimizerParameters.h>

namespace wmtk::components::tetwild {
/// The fields shared with triwild and simwild live in wmtk::OptimizerParameters.
struct Parameters : public wmtk::OptimizerParameters
{
    /// Order-2 (open boundary / non-manifold edge) envelope thickness, as a fraction of the
    /// surface envelope's. Deliberately below 1 where the surface envelope uses the full eps;
    /// see the doc on /order2_envelope_ratio in the spec for the measurement behind 0.5.
    double order2_envelope_ratio = 0.5;
    Vector3d min = Vector3d::Zero();
    Vector3d max = Vector3d::Ones();
    Vector3d box_min = Vector3d::Zero();
    Vector3d box_max = Vector3d::Ones();

    // Allow the edge swaps (3->2, 4-4, 5-6) to operate on surface edges as a
    // topology-preserving surface diagonal flip, instead of forbidding them
    // outright. Enabled by default; can be turned off to reproduce the old
    // surface-frozen behavior for A/B testing.
    bool allow_surface_swap = true;
    // Expensive debug check: verify the global surface topology signature
    // (connected components, Euler characteristic, boundary loops) is unchanged
    // across each swap pass. Off by default (used by tests / debugging).
    bool check_surface_topology = false;

    /**
     * How many smoothing passes each optimization iteration runs.
     *
     * This is ops[3] in local_operations({{split, collapse, swap, smooth}}), which was
     * hard-coded to 1. Smoothing is the only phase that improves element quality without
     * changing connectivity, so on meshes where split/collapse/swap have run out of useful
     * moves it is the only thing left that can lower the energy.
     */
    int num_smoothing_passes = 2;

    // Interleave smoothing between the topology passes instead of running it all at the end
    // of the iteration. With this on, one iteration is
    //     split   + interleaved_smoothing_passes smoothing passes
    //     collapse + ...
    //     swaps    + ...
    // rather than split, collapse, swaps, then num_smoothing_passes passes. Smoothing is the
    // only phase that improves quality without changing connectivity, so giving each topology
    // pass a chance to be relaxed before the next one runs may keep the optimizer off the
    // plateaus where split, collapse and swap simply undo each other.
    bool interleaved_smoothing = true;
    int interleaved_smoothing_passes = 1;

    void init(const Vector3d& min_, const Vector3d& max_)
    {
        min = min_;
        max = max_;
        init_lengths_from_diagonal((max - min).norm());
        l_min = eps;
    }
    void init(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces)
    {
        Vector3d min_, max_;
        for (size_t i = 0; i < vertices.size(); i++) {
            if (i == 0) {
                min_ = vertices[i];
                max_ = vertices[i];
                continue;
            }
            for (int j = 0; j < 3; j++) {
                if (vertices[i][j] < min_[j]) min_[j] = vertices[i][j];
                if (vertices[i][j] > max_[j]) max_[j] = vertices[i][j];
            }
        }

        init(min_, max_);
    }
};
} // namespace wmtk::components::tetwild
