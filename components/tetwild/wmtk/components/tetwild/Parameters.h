#pragma once

#include <wmtk/OptimizerParameters.h>

namespace wmtk::components::tetwild {
/// The fields shared with triwild and simwild live in wmtk::OptimizerParameters.
struct Parameters : public wmtk::OptimizerParameters
{
    /// Order-2 (open boundary) tube thickness, as a fraction of the surface envelope's eps. 1:
    /// every held set stays within the one eps; see /order2_envelope_ratio in the spec for the
    /// sweep behind the default.
    double order2_envelope_ratio = 1.0;
    /// Feature-curve tube thickness, and the feature-point anchors' radius, as a fraction of
    /// eps. 1, as order2_envelope_ratio.
    double feature_envelope_ratio = 1.0;
    /// Keep the 0-dimensional features -- input points, and feature-curve endpoints (and
    /// junctions, see allow_junction_cleanup) -- within m_feature_eps of where the input
    /// put them. Mirrors triwild's parameter of the same name.
    bool preserve_feature_points = true;
    /// Anchor only the endpoints of open feature curves, letting junctions merge. Mirrors
    /// triwild: the erosion the anchor exists for is an ENDPOINT property.
    bool allow_junction_cleanup = true;
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
