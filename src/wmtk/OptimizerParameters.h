#pragma once

#include <limits>

namespace wmtk {

/**
 * @brief The parameters tetwild, triwild and simwild all share.
 *
 * Moved here from tetwild's Parameters.h, which had the fullest field set and the
 * measurements written down. Each application derives its own Parameters from this and adds
 * what is genuinely its own.
 *
 * Two things deliberately stay in the applications:
 *
 *   * **The bounding box.** Its type differs by dimension (Vector3d / Vector2d / VectorXd) and,
 *     more importantly, so does its MEANING: tetwild's box_min/box_max are the *padded Delaunay
 *     box* while simwild's are the raw input bbox, and both are compared with `==` against
 *     vertex coordinates to tag bbox faces. Sharing one field would silently break the tagging
 *     in one of them. tetwild additionally keeps `min`/`max` for the input bbox.
 *   * **`l_min`.** tetwild and triwild set it to `eps`, simwild to `0.5 * eps`.
 *
 * Defaults here are tetwild's and triwild's shared defaults; simwild overrides the four it
 * deliberately differs on in its own struct, where the divergence is visible.
 */
struct OptimizerParameters
{
    double epsr = 1e-3; // relative error bound (wrt diagonal)
    double eps = -1.; // absolute error bound
    double lr = 5e-2; // target edge length (relative)
    double l = -1.;
    double l_min = -1;
    double diag_l = -1.;

    bool preserve_topology = false;

    /**
     * Incident-cell count above which a link vertex accepts only one valence-increasing split
     * per pass, or 0 to disable the gate. Shared by the 2D and 3D Wild optimizers; SimWild uses
     * the same protection so tag-homogeneous runs follow the Wild path.
     */
    int split_high_valence_threshold = 200;

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
    // Number of worst cells (by energy) whose neighborhoods are refined.
    int stuck_refine_num_worst = 0;
    // Graph rings around each worst cell's vertices included in the refinement.
    int stuck_refine_rings = 0;
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
    // Force-split: when the max energy stalls, split each worst cell's longest edge
    // once, bypassing the split length gate. This unsticks a sliver whose edges are
    // too short to be split-eligible, WITHOUT touching the sizing field (which the
    // *factor ratchet above still drives). Adds at most one split per worst cell per
    // stall, so it does not bloat the element count.
    bool stuck_refine_force_split = true;

    /**
     * Force-split only a cell that is too LARGE for its sizing field.
     *
     * Force-split exists to unstick a sliver, but it cannot: AMIPS is scale invariant, so
     * subdividing a badly SHAPED cell yields two badly shaped cells of the same energy. The
     * cell stays the worst one, is force-split again on the next stall, and its longest edge
     * halves every time -- a ratchet with no exit. On Thingi10K 243014 that drove a legitimate
     * 0.155 surface edge, the finest the simplified input has, down to 1.0e-4 in about eleven
     * firings, ~1500x below anything in the input.
     *
     * Refinement only helps a cell that is too big for its target length, so require that.
     * Measured serially on 243014 over 25 iterations: off gives final max energy 41.46 with a
     * 1.02e-04 minimum edge and 0.373% of edges below 1e-3; on gives 20.87, 8.15e-04 and
     * 0.019%. Without it the mesh reaches 20.87 then degrades to 41.46; with it 20.87 holds.
     */
    bool stuck_refine_force_split_oversized_only = true;

    // ---- Skip good regions ----------------------------------------------
    // Only smooth vertices incident to a cell whose energy is >=
    // skip_good_regions_margin * stop_energy. Smoothing a vertex surrounded by
    // good cells does nothing, so skipping it is free (14-16x faster smooth
    // passes). Only smoothing is gated: gating the topology/sizing ops
    // (split/collapse/swap) starves the optimizer and blows up the element
    // count, so those always run over the whole mesh.
    bool skip_good_regions = false;
    // Safety margin on the "active" threshold: a cell is active when its energy is >= this
    // fraction of stop_energy, so vertices near cells sitting just below the target are
    // still smoothed.
    double skip_good_regions_margin = 0.9;

    double splitting_l2 = -1.; // the lower bound length (squared) for edge split
    double collapsing_l2 =
        std::numeric_limits<double>::max(); // the upper bound length (squared) for edge collapse

    double stop_energy = 100;

    /**
     * Relative weight of the AMIPS (quality) term against the envelope (stay-on-surface)
     * term during smoothing. w_envelope is derived as 1 - w_amips in each mesh's
     * constructor, so the small default means the envelope dominates and AMIPS acts as a
     * light quality preference.
     */
    double w_amips = 1e-4;
    /**
     * Place surface vertices by an unconstrained solve plus a projected line search instead
     * of by the envelope pull term. See SmoothVertexOptions::project_line_search.
     */
    bool project_line_search = true;
    /// Bisections tried before the projected search gives up. See SmoothVertexOptions.
    int project_line_search_steps = 12;
    /// Partial-projection bisections tried after it gives up; 0 disables that pass.
    int project_line_search_nested_steps = 0;
    double w_envelope = 1. - 1e-4; // derived; not read from json

    /// Number and placement of smoothing passes in the shared Wild optimization driver.
    int num_smoothing_passes = 2;
    bool interleaved_smoothing = true;
    int interleaved_smoothing_passes = 1;

    bool debug_output = false;
    bool perform_sanity_checks = false;

    /**
     * @brief Derive the edge-length and envelope quantities from the bounding-box diagonal.
     *
     * The identical algebra in all three applications' init(). The caller computes diag_l from
     * its own bounding box (whose type and meaning differ -- see the class comment) and sets
     * its own l_min afterwards.
     */
    void init_lengths_from_diagonal(const double diag)
    {
        diag_l = diag;
        if (l > 0) {
            lr = l / diag_l;
        } else {
            l = lr * diag_l;
        }
        splitting_l2 = l * l * (16 / 9.);
        collapsing_l2 = l * l * (16 / 25.);

        if (eps > 0) {
            epsr = eps / diag_l;
        } else {
            eps = epsr * diag_l;
        }
    }
};

} // namespace wmtk
