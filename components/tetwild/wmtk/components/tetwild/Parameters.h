#pragma once

namespace wmtk::components::tetwild {
struct Parameters
{
    double epsr = 2e-3; // relative error bound (wrt diagonal)
    double eps = -1.; // absolute error bound
    double lr = 5e-2; // target edge length (relative)
    double l = -1.;
    double l_min = -1;
    double diag_l = -1.;
    Vector3d min = Vector3d::Zero();
    Vector3d max = Vector3d::Ones();
    Vector3d box_min = Vector3d::Zero();
    Vector3d box_max = Vector3d::Ones();
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
    // When the max energy stops improving, aggressively refine the sizing field
    // around the worst elements so subsequent split/smooth/swap get more DOF to
    // untangle stubborn (surface) slivers. Replaces the old global
    // adjust_sizing_field mechanism.
    bool stuck_refine = true;
    // Trigger threshold: fire when the max energy did not improve by more than
    // this *fraction* since the previous iteration, i.e. refine when
    // (prev_max - max) <= stall_eps * prev_max. 0 => only when it does not
    // improve at all (or gets worse).
    double stuck_refine_stall_eps = 0.01;
    // Cooldown: after a refinement, skip this many improvement iterations before
    // refining again, so the operations get full passes to act on the new sizing
    // field before more refinement is added. 0 => may refine every iteration.
    int stuck_refine_cooldown = 2;
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

    double splitting_l2 = -1.; // the lower bound length (squared) for edge split
    double collapsing_l2 =
        std::numeric_limits<double>::max(); // the upper bound length (squared) for edge collapse

    double stop_energy = 10;

    bool debug_output = false;
    bool perform_sanity_checks = false;

    void init(const Vector3d& min_, const Vector3d& max_)
    {
        min = min_;
        max = max_;
        diag_l = (max - min).norm();
        if (l > 0)
            lr = l / diag_l;
        else
            l = lr * diag_l;
        splitting_l2 = l * l * (16 / 9.);
        collapsing_l2 = l * l * (16 / 25.);

        if (eps > 0)
            epsr = eps / diag_l;
        else
            eps = epsr * diag_l;

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
