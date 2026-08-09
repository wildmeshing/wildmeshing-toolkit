#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::components::simwild {
struct Parameters
{
    // parameters set by user
    double epsr = 2e-3; // relative error bound (wrt diagonal)
    double eps = -1.; // absolute error bound
    double lr = 5e-2; // target edge length (relative)
    double l = -1.; // target edge length (absolute)
    double l_min = -1;
    bool preserve_topology = false;
    std::string output_path;

    // Allow the 3->2 edge swap to operate on surface edges (a surface diagonal
    // flip) instead of forbidding them outright. Enabled by default; can be
    // turned off to reproduce the old surface-frozen behavior for A/B testing.
    bool allow_surface_swap = true;
    // Expensive debug check: verify the global surface topology signature
    // (connected components, Euler characteristic, boundary loops) is unchanged
    // across each swap pass. Off by default (used by tests / debugging).
    bool check_surface_topology = false;

    // ---- Stuck-element sizing refinement --------------------------------
    // Trigger threshold: fire when the last iteration's improvement is small compared
    // with the distance the max energy still has to cover, i.e. refine when
    //     (prev_max - max) <= stall_eps * (max - target).
    // Equivalently: refine unless the mesh is on course to reach the target within
    // about 1/stall_eps more iterations. 0 => only when it does not improve at all.
    //
    // The denominator is the remaining distance, not prev_max, and that is the whole
    // point. Measured against prev_max, a mesh grinding down at a steady 1.3% per
    // iteration toward a target far below clears a 1% bar every single iteration and so
    // never looks stalled -- even though at that rate it needs on the order of a hundred
    // more iterations and the operations have in fact deadlocked. The escape hatch then
    // never fires, which is exactly the regime it exists for.
    double stuck_refine_stall_eps = 0.1;
    // Cooldown: after a refinement, skip this many improvement iterations before
    // refining again, so the operations get full passes to act on the new sizing
    // field before more refinement is added. 0 => may refine every iteration.
    //
    // 0 by default: measured over 468 triwild20k models, a cooldown of 1 costs ~13% wall
    // time for exactly the same mesh sizes (identical median and p90 vertex counts). The
    // idea that the operations need an idle iteration to act on the new field does not
    // survive contact with the data -- the trigger already declines to fire while the mesh
    // is converging, so a separate cooldown only delays the next escape.
    int stuck_refine_cooldown = 0;
    // Number of worst tets (by energy) whose neighborhoods are refined.
    int stuck_refine_num_worst = 50;
    // Graph rings around each worst tet's vertices included in the refinement.
    int stuck_refine_rings = 3;
    // Multiplicative reduction of m_sizing_scalar per refinement (0.5 => /2).
    double stuck_refine_factor = 0.5;
    // Lower bound on m_sizing_scalar. Much smaller than the old l_min/l floor;
    // still far above the position-rounding scale so it stays numerically safe.
    double stuck_refine_min_scalar = 1e-3;
    // Gradation cap for the monotone sizing smoothing: neighboring sizings may
    // differ by at most this factor. The smoothing only ever *lowers* sizings
    // (spreads refinement outward), never raises the refined values, avoiding
    // sharp resolution jumps that make operations ill-conditioned.
    double stuck_refine_gradation = 2.0;
    // Force-split: when the max energy stalls, split each worst tet's longest edge
    // once, bypassing the split length gate. This unsticks a sliver whose edges are
    // too short to be split-eligible, WITHOUT touching the sizing field (which the
    // *factor ratchet above still drives). Adds at most one split per worst tet per
    // stall, so it does not bloat the tet count.
    bool stuck_refine_force_split = true;
    // When a split's rounded (double) midpoint would invert an incident tet, the
    // split is normally rejected. If this is on, splits of edges that belong to
    // the current worst-tet set (the seeds used by refine_sizing_around_worst)
    // instead fall back to the EXACT rational midpoint (never inverts) and keep
    // the new vertex un-rounded, so the worst region can still be refined.

    // ---- Skip good regions ----------------------------------------------
    // Only smooth vertices incident to a tet whose energy is >=
    // skip_good_regions_margin * stop_energy. Smoothing a vertex surrounded by
    // good tets does nothing, so skipping it is free (14-16x faster smooth
    // passes). Only smoothing is gated: gating the topology/sizing ops
    // (split/collapse/swap) starves the optimizer and blows up the element
    // count, so those always run over the whole mesh.
    bool skip_good_regions = true;
    // Safety margin on the "active" threshold: a tet is active when its energy
    // (cbrt of m_quality) is >= this fraction of stop_energy, so vertices near
    // tets sitting just below the target are still smoothed.
    double skip_good_regions_margin = 0.9;


    double epsr_simplify = 2e-3; // relative error bound (wrt diagonal) for simplification
    double eps_simplify = -1.; // absolute error bound for simplification

    // parameters set in `init` function based on mesh bbox
    double diag_l = -1.;
    VectorXd box_min;
    VectorXd box_max;
    double splitting_l2 = -1.; // the lower bound length (squared) for edge split
    double collapsing_l2 =
        std::numeric_limits<double>::max(); // the upper bound length (squared) for edge collapse

    double stop_energy = 10;
    bool stop_at_float = false;

    bool debug_output = false;
    bool perform_sanity_checks = false;

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

    // weighting terms for the optimization
    double w_amips = 1e-4;
    double w_envelope = 0;

    std::string operation = "remeshing";

    bool skip_simplify = true;
    bool use_sample_envelope = false;
    int NUM_THREADS = 0;
    int max_its = 80;
    bool write_vtu = false;
    bool write_envelope = true;

    Parameters() = default;

    Parameters(const nlohmann::json& json_params)
    {
        output_path = json_params["output"];
        skip_simplify = json_params["skip_simplify"];
        use_sample_envelope = json_params["use_sample_envelope"];
        NUM_THREADS = json_params["num_threads"];
        max_its = json_params["max_iterations"];
        write_vtu = json_params["write_vtu"];
        write_envelope = json_params["write_envelope"];

        epsr = json_params["eps_rel"];
        eps = json_params["eps"];
        lr = json_params["length_rel"];
        l = json_params["length"];
        stop_energy = json_params["stop_energy"];
        stop_at_float = json_params["stop_at_float"];
        preserve_topology = json_params["preserve_topology"];

        epsr_simplify = json_params["eps_simplify_rel"];
        eps_simplify = json_params["eps_simplify"];

        w_amips = json_params["w_amips"];

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
        diag_l = (box_max - box_min).norm();
        if (l > 0)
            lr = l / diag_l;
        else
            l = lr * diag_l;
        splitting_l2 = l * l * (16 / 9.);
        collapsing_l2 = l * l * (16 / 25.);

        if (eps > 0) {
            epsr = eps / diag_l;
        } else {
            eps = epsr * diag_l;
        }

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
