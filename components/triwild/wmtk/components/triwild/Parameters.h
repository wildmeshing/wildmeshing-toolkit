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

    /**
     * Incident-triangle count above which a vertex is treated as pathological, or 0 to
     * disable the gate.
     *
     * The 2D counterpart of tetwild's gate. Splitting edge (a,b) leaves a's and b's own
     * counts unchanged and adds one to every vertex in the edge's link, so the gate is
     * applied to the link, not the endpoints -- but in 2D that link is one or two vertices,
     * not a whole ring, so the runaway this guards against is far less likely here.
     */
    int split_high_valence_threshold = 200;

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
    //     split    + interleaved_smoothing_passes smoothing passes
    //     collapse + ...
    //     swaps    + ...
    // rather than split, collapse, swaps, then num_smoothing_passes passes. Smoothing is the
    // only phase that improves quality without changing connectivity, so giving each topology
    // pass a chance to be relaxed before the next one runs may keep the optimizer off the
    // plateaus where split, collapse and swap simply undo each other.
    bool interleaved_smoothing = true;
    int interleaved_smoothing_passes = 1;

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

        debug_output = json_params["DEBUG_output"];
        perform_sanity_checks = json_params["DEBUG_sanity_checks"];
        check_envelope_at_init = json_params["DEBUG_envelope_sanity_check"];
        use_sample_envelope = json_params["use_sample_envelope"];

        split_high_valence_threshold = json_params["split_high_valence_threshold"];
        w_amips = json_params["w_amips"];
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
