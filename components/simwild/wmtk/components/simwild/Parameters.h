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
    // Trigger threshold: fire when the max energy did not improve by more than
    // this *fraction* since the previous iteration, i.e. refine when
    // (prev_max - max) <= stall_eps * prev_max. 0 => only when it does not
    // improve at all (or gets worse).
    double stuck_refine_stall_eps = 0.01;
    // Cooldown: after a refinement, skip this many improvement iterations before
    // refining again, so the operations get full passes to act on the new sizing
    // field before more refinement is added. 0 => may refine every iteration.
    int stuck_refine_cooldown = 1;
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
    bool stuck_refine_rational_split = true;

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
    // <= 0: ambient follows stop_energy. Otherwise pure-ambient faces use
    // this (typically looser) stop: quality where the physics lives (bodies,
    // interfaces), tolerance in the ambient filler. Currently 2D only.
    double stop_energy_ambient = -1;
    // tag names whose cells count as ambient space when ALL of a cell's
    // tags are in this list (plus ambient itself) — e.g. user-added
    // primitives like box_0: {box_0} is ambient-like, {box_0, dragon} is
    // body. Consulted wherever ambient is discriminated (stop_energy_ambient,
    // body-energy logs, sizing-adjust seeding).
    std::vector<std::string> ambient_like_tags;
    // sizing-adjust seeding threshold as a fraction of stop_energy. The
    // legacy value 0.8 (TetWild hysteresis) seeds refinement around faces
    // that already satisfy stop_energy — feature-pinned junction faces live
    // in (0.8*stop, stop] forever and cause a mesh-wide refinement spiral
    // when the improvement loop stalls. 1.0 = only seed failing faces.
    double adjust_filter_rel = 1.0;
    bool stop_at_float = false;

    bool debug_output = false;
    bool perform_sanity_checks = false;

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
        stop_energy_ambient = json_params["stop_energy_ambient"];
        adjust_filter_rel = json_params["adjust_filter_rel"];
        ambient_like_tags = json_params["ambient_like_tags"].get<std::vector<std::string>>();
        stop_at_float = json_params["stop_at_float"];
        preserve_topology = json_params["preserve_topology"];

        epsr_simplify = json_params["eps_simplify_rel"];
        eps_simplify = json_params["eps_simplify"];

        w_amips = json_params["w_amips"];

        debug_output = json_params["DEBUG_output"];
        perform_sanity_checks = json_params["DEBUG_sanity_checks"];

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
        stuck_refine_rational_split = json_params["stuck_refine_rational_split"];

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
