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
     * Radius of the anchors of the polyline endpoints, as a fraction of eps. 1: every held set
     * stays within the one eps, as in tetwild and simwild.
     *
     * Without the anchors the collapse pass deletes open polylines outright: a polyline erodes
     * into its own tip until one segment is left, and that segment has a feature at both ends,
     * which nothing else refuses. Always on; until 2026-10 preserve_feature_points could turn
     * them off, to reproduce the behaviour before them.
     */
    double order2_envelope_ratio = 1.0;
    /// Radius of the anchors of the input free points, as a fraction of eps. 1, as
    /// order2_envelope_ratio.
    double feature_envelope_ratio = 1.0;

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
     * acceptance radius to eps/2 (see Envelope::eps2_edge). The exact one asks whether
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
        optimize_envelope_around_simplified = json_params["optimize_envelope_around_simplified"];
        order2_envelope_ratio = json_params["order2_envelope_ratio"];
        feature_envelope_ratio = json_params["feature_envelope_ratio"];

        debug_output = json_params["DEBUG_output"];
        perform_sanity_checks = json_params["DEBUG_sanity_checks"];
        check_envelope_at_init = json_params["DEBUG_envelope_sanity_check"];
        use_sample_envelope = json_params["use_sample_envelope"];

        split_high_valence_threshold = json_params["split_high_valence_threshold"];
        w_amips = json_params["w_amips"];
        smoothing_mode = json_params["smoothing_mode"];
        project_line_search_steps = json_params["project_line_search_steps"];
        project_line_search_nested_steps = json_params["project_line_search_nested_steps"];
        num_smoothing_passes = json_params["num_smoothing_passes"];
        interleaved_smoothing = json_params["interleaved_smoothing"];
        interleaved_smoothing_passes = json_params["interleaved_smoothing_passes"];
        colored_smoothing = json_params["colored_smoothing"];

        // Coarsening pass.
        coarsen_pass = json_params["coarsen_pass"];
        coarsen_unbounded = json_params["coarsen_unbounded"];
        coarsen_local_smoothing_passes = json_params["coarsen_local_smoothing_passes"];
        coarsen_smooth_ring = json_params["coarsen_smooth_ring"];
        coarsen_global_smoothing_passes = json_params["coarsen_global_smoothing_passes"];
        coarsen_max_rounds = json_params["coarsen_max_rounds"];
        coarsen_max_inner_passes = json_params["coarsen_max_inner_passes"];

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
