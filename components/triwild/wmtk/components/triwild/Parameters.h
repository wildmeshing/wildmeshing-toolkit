#pragma once

#include <wmtk/OptimizerParameters.h>

#include <nlohmann/json.hpp>

namespace wmtk::components::triwild {
/// The fields shared with tetwild and simwild live in wmtk::OptimizerParameters.
struct Parameters : public wmtk::OptimizerParameters
{
    Vector2d box_min = Vector2d::Zero();
    Vector2d box_max = Vector2d::Ones();

    /**
     * Keep the curve network's 0-dimensional features -- open polyline endpoints and
     * junctions -- within eps of where the arrangement put them.
     *
     * Without it the collapse pass deletes open polylines outright: a polyline erodes into
     * its own tip until one segment is left, and that segment has a feature at both ends,
     * which nothing else refuses. Off only for A/B against the old behaviour.
     */
    bool preserve_feature_points = true;

    /**
     * Let the operations clean up junctions, anchoring only open-polyline endpoints.
     *
     * The erosion argument above is about ENDPOINTS: a polyline eats its own tip. A junction
     * -- valence >= 3 in the constrained edges -- cannot erode a curve away, because every
     * curve through it stays a constrained edge and the envelope still holds it within eps.
     * Anchoring it buys little and costs a great deal, because on a self-intersecting input
     * nearly every arrangement vertex is a crossing: on 2D model 242427, 79615 of 87610
     * vertices are junctions, so almost every edge joins two of them and collapse -- the only
     * operation that removes a bad element outright -- is refused everywhere. That model then
     * never leaves MAX_ENERGY: it sat at 1e50 for the whole run while the sizing field
     * saturated and the split pass grew it from 13k to 621k vertices.
     *
     * With junctions free it converges to max energy 10.48, fully rounded, in 12 iterations.
     * Measured no change on the two models the endpoint guard was introduced for (215292 and
     * 134005): identical energies, iteration counts and Euler characteristics either way.
     *
     * Ignored when preserve_feature_points is off, which already anchors nothing.
     */
    bool allow_junction_cleanup = true;

    /**
     * Verify at init that every constrained edge starts inside the envelope.
     *
     * The invariant is real -- the envelope is built around the *input* curves while the
     * constrained edges come from the simplified ones, so it checks that the simplification
     * stayed inside its share of eps -- but it costs one sampled segment query per
     * constrained edge, serially. On a 3.5M-edge input that is 22s locally and 63s on a
     * slower machine, paid on every run to catch something that has fired once in 15665
     * models. Off by default; turn it on when changing the simplification or the envelope.
     */
    bool check_envelope_at_init = false;

    /**
     * Use the sampled envelope rather than the exact one for the mesh optimization.
     *
     * The two answer a different question. The sampled test places points along the query
     * segment and asks whether each is within eps of the input; it therefore cannot see
     * anything that happens between two samples, and pays for that by shrinking its
     * acceptance radius to eps/2 (see SampleEnvelope::eps2_edge). The exact one asks whether
     * the segment is covered by the union of the eps-rectangles around the input segments and
     * decides it without sampling, so it uses the full eps and answers a strictly sharper
     * question -- notably it rejects a segment that bridges a gap between two input curves,
     * which the sampled test accepts whenever the gap is narrow enough to fall between samples.
     *
     * Exact by default, matching tetwild.
     */
    bool use_sample_envelope = false;

    Parameters() = default;

    /**
     * @brief Read every optimizer knob out of the (defaults-injected) JSON.
     *
     * Mirrors simwild's Parameters(json) so the three applications stay in step.
     */
    Parameters(const nlohmann::json& json_params)
    {
        epsr = json_params["eps_rel"];
        lr = json_params["length_rel"];
        stop_energy = json_params["stop_energy"];
        preserve_topology = json_params["preserve_topology"];
        preserve_feature_points = json_params["preserve_feature_points"];
        allow_junction_cleanup = json_params["allow_junction_cleanup"];

        debug_output = json_params["DEBUG_output"];
        perform_sanity_checks = json_params["DEBUG_sanity_checks"];
        check_envelope_at_init = json_params["DEBUG_envelope_sanity_check"];
        use_sample_envelope = json_params["use_sample_envelope"];

        split_high_valence_threshold = json_params["split_high_valence_threshold"];
        w_amips = json_params["w_amips"];
        spring_pull = json_params["spring_pull"];
        num_smoothing_passes = json_params["num_smoothing_passes"];
        interleaved_smoothing = json_params["interleaved_smoothing"];
        interleaved_smoothing_passes = json_params["interleaved_smoothing_passes"];

        // Stuck-element sizing refinement.
        stuck_refine_stall_eps = json_params["stuck_refine_stall_eps"];
        stuck_refine_cooldown = json_params["stuck_refine_cooldown"];
        stuck_refine_num_worst = json_params["stuck_refine_num_worst"];
        stuck_refine_rings = json_params["stuck_refine_rings"];
        stuck_refine_factor = json_params["stuck_refine_factor"];
        stuck_refine_min_scalar = json_params["stuck_refine_min_scalar"];
        stuck_refine_gradation = json_params["stuck_refine_gradation"];
        stuck_refine_force_split = json_params["stuck_refine_force_split"];

        // Skip good regions.
        skip_good_regions = json_params["skip_good_regions"];
        skip_good_regions_margin = json_params["skip_good_regions_margin"];
    }

    void init(const Vector2d& min_, const Vector2d& max_)
    {
        box_min = min_;
        box_max = max_;
        init_lengths_from_diagonal((box_max - box_min).norm());
        l_min = eps;
    }
    void init(
        const std::vector<Vector2d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces)
    {
        Vector2d min_, max_;
        for (size_t i = 0; i < vertices.size(); i++) {
            if (i == 0) {
                min_ = vertices[i];
                max_ = vertices[i];
                continue;
            }
            for (int j = 0; j < 2; j++) {
                if (vertices[i][j] < min_[j]) {
                    min_[j] = vertices[i][j];
                }
                if (vertices[i][j] > max_[j]) {
                    max_[j] = vertices[i][j];
                }
            }
        }

        init(min_, max_);
    }
};
} // namespace wmtk::components::triwild
