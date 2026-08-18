#pragma once

#include <wmtk/OptimizerParameters.h>
#include <nlohmann/json.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::components::simwild {
/// The fields shared with tetwild and triwild live in wmtk::OptimizerParameters.
struct Parameters : public wmtk::OptimizerParameters
{
    /**
     * The four shared fields where simwild deliberately differs from tetwild/triwild. Set here
     * rather than in the base so the divergence is visible in one place; each matches the
     * default in simwild_spec.json.
     */
    Parameters()
    {
        epsr = 2e-3; // tetwild/triwild: 1e-3
        stop_energy = 10; // tetwild/triwild: 100
        preserve_topology = true; // tetwild/triwild: false
        w_envelope = 0; // derived as 1 - w_amips in the mesh constructor
    }

    std::string output_path;

    // Allow the 3->2 edge swap to operate on surface edges (a surface diagonal
    // flip) instead of forbidding them outright. Enabled by default; can be
    // turned off to reproduce the old surface-frozen behavior for A/B testing.
    bool allow_surface_swap = true;
    // Expensive debug check: verify the global surface topology signature
    // (connected components, Euler characteristic, boundary loops) is unchanged
    // across each swap pass. Off by default (used by tests / debugging).
    bool check_surface_topology = false;


    double epsr_simplify = 2e-4; // relative error bound (wrt diagonal) for simplification
    /// Order-2 (open boundary / non-manifold edge) envelope thickness, as a fraction of the
    /// surface envelope's. Deliberately below 1 where the surface envelope uses the full eps;
    /// see the doc on /order2_envelope_ratio in the spec for the measurement behind 0.5.
    double order2_envelope_ratio = 0.5;
    double eps_simplify = -1.; // absolute error bound for simplification
    VectorXd box_min;
    VectorXd box_max;
    bool stop_at_float = false;

    /**
     * Verify at init that every surface edge starts inside the envelope (2D remeshing only).
     *
     * The same check triwild carries, and the same trade: the invariant is real, but it
     * costs one sampled segment query per surface edge, serially, on every run. Measured in
     * triwild's 2D sweep at 22s on a 3.5M-edge input and 5m07s on a 7.6M-edge one -- 8.5% of
     * that model's entire budget, spent before the first iteration. Off by default.
     *
     * The 3D counterpart in VolumemesherInsertion is deliberately NOT gated by this: it is
     * not a check but a decision, feeding the "rebuild the envelope from tet tags" branch.
     */
    bool check_envelope_at_init = false;

    std::string operation = "remeshing";

    bool skip_simplify = false;
    bool use_sample_envelope = false;
    int NUM_THREADS = 0;
    bool write_vtu = false;
    bool write_envelope = true;


    Parameters(const nlohmann::json& json_params)
    {
        output_path = json_params["output"];
        skip_simplify = json_params["skip_simplify"];
        use_sample_envelope = json_params["use_sample_envelope"];
        NUM_THREADS = json_params["num_threads"];
        write_vtu = json_params["write_vtu"];
        write_envelope = json_params["write_envelope"];

        epsr = json_params["eps_rel"];
        eps = json_params["eps"];
        lr = json_params["length_rel"];
        l = json_params["length"];
        stop_energy = json_params["stop_energy"];
        stop_at_float = json_params["stop_at_float"];
        preserve_topology = json_params["preserve_topology"];
        optimize_envelope_around_simplified = json_params["optimize_envelope_around_simplified"];

        epsr_simplify = json_params["eps_simplify_rel"];
        order2_envelope_ratio = json_params["order2_envelope_ratio"];
        eps_simplify = json_params["eps_simplify"];

        w_amips = json_params["w_amips"];
        smoothing_mode = json_params["smoothing_mode"];
        project_line_search_steps = json_params["project_line_search_steps"];
        project_line_search_nested_steps = json_params["project_line_search_nested_steps"];
        num_smoothing_passes = json_params["num_smoothing_passes"];
        interleaved_smoothing = json_params["interleaved_smoothing"];
        interleaved_smoothing_passes = json_params["interleaved_smoothing_passes"];

        // Coarsening pass. Implemented on the shared 2D optimizer, so it applies to simwild's
        // 2D mesh and is inert on its 3D one.
        coarsen_pass = json_params["coarsen_pass"];
        coarsen_unbounded = json_params["coarsen_unbounded"];
        coarsen_local_smoothing_passes = json_params["coarsen_local_smoothing_passes"];
        coarsen_smooth_ring = json_params["coarsen_smooth_ring"];
        coarsen_global_smoothing_passes = json_params["coarsen_global_smoothing_passes"];
        coarsen_max_rounds = json_params["coarsen_max_rounds"];
        coarsen_max_inner_passes = json_params["coarsen_max_inner_passes"];
        collapse_quality_margin = json_params["collapse_quality_margin"];
        debug_edge_length_match = json_params["debug_edge_length_match"];
        sizing_field_from_features = json_params["sizing_field_from_features"];
        sizing_field_min_eps_ratio = json_params["sizing_field_min_eps_ratio"];

        debug_output = json_params["DEBUG_output"];
        perform_sanity_checks = json_params["DEBUG_sanity_checks"];
        check_envelope_at_init = json_params["DEBUG_envelope_sanity_check"];

        allow_surface_swap = json_params["allow_surface_swap"];
        check_surface_topology = json_params["check_surface_topology"];

        // Stuck-element sizing refinement.
        stuck_refine_stall_eps = json_params["stuck_refine_stall_eps"];
        stuck_refine_cooldown = json_params["stuck_refine_cooldown"];
        stuck_refine_num_worst = json_params["stuck_refine_num_worst"];
        stuck_refine_rings = json_params["stuck_refine_rings"];
        stuck_refine_factor = json_params["stuck_refine_factor"];
        stuck_refine_force_split = json_params["stuck_refine_force_split"];
        stuck_refine_min_scalar = json_params["stuck_refine_min_scalar"];
        stuck_refine_gradation = json_params["stuck_refine_gradation"];

        // Skip good regions.
        skip_good_regions = json_params["skip_good_regions"];
        skip_good_regions_margin = json_params["skip_good_regions_margin"];

        operation = json_params["operation"];
    }

    void init(const VectorXd& min_, const VectorXd& max_)
    {
        box_min = min_;
        box_max = max_;
        init_lengths_from_diagonal((box_max - box_min).norm());

        if (eps_simplify > 0) {
            epsr_simplify = eps_simplify / diag_l;
        } else {
            eps_simplify = epsr_simplify * diag_l;
        }

        l_min = 0.5 * eps;
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
} // namespace wmtk::components::simwild
