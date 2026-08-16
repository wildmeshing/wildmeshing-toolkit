#pragma once

#include <limits>
#include <string>

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
     * Build the optimizer's envelope around the SIMPLIFIED geometry at the REMAINING
     * tolerance (eps - simplify_eps), instead of around the original input at the full eps.
     *
     * The deviation budget is identical by the triangle inequality: the simplification is
     * already within simplify_eps of the input, so anything within (eps - simplify_eps) of the
     * simplification is within eps of the input. What changes is where the geometry STARTS.
     * The envelope is a hard veto, not a penalty, so a surface handed to the optimizer close
     * to the boundary has most of its moves refused; built this way it starts at the centre,
     * with the whole radius available in every direction.
     *
     * That headroom is load-bearing: deliberately starving it on tetwild (simplify_envelope_
     * ratio 0.95, which leaves the simplification free to use nearly the whole tolerance) was
     * enough to turn a converging run into a diverging one on Thingi10K 1368052.
     *
     * The cost is elements. Holding the surface within eps/2 of the simplified geometry is
     * stricter than eps of the input wherever the simplification smoothed detail away, so
     * fewer coarsening collapses are allowed: measured on 106838 the output went from 200k to
     * 560k tets and on 116060 from 133k to 267k, at unchanged final quality. Off by default
     * for that reason.
     *
     * SimWild's 3D mesh already rebuilds its envelope around the simplified surface; there
     * this flag only narrows the radius to the remaining budget, which is the part it was
     * missing. SimWild's 2D mesh has no simplification stage, so the flag does not apply.
     */
    bool optimize_envelope_around_simplified = false;

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
    //
    // This deliberately applies to EVERY worst cell, including one already at its target
    // size. There used to be a stuck_refine_force_split_oversized_only gate that skipped
    // those, on the argument that AMIPS is scale invariant so subdividing a badly shaped
    // cell yields two badly shaped cells. The argument is sound about the cell itself and
    // wrong about the outcome: force-splitting it also refines its NEIGHBOURHOOD, and that
    // is what breaks a deadlocked configuration open. On Thingi10K 46024 -- the one model of
    // 10,000 in the sweep that finished above stop_energy -- the gate refused ~2000 tets on
    // every stall, the max energy froze at iteration 3 and stayed identical to 15 significant
    // figures for the remaining 77 iterations (4.6 h). Without the gate the same model
    // converges to 9.98 in 14 iterations and 11 minutes.
    bool stuck_refine_force_split = true;


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
    /// "projected" or "exact"; see SmoothVertexOptions::SmoothingMode.
    std::string smoothing_mode = "projected";
    /// Bisections tried before the projected search gives up. See SmoothVertexOptions.
    int project_line_search_steps = 12;
    /// Partial-projection bisections tried after it gives up; 0 disables that pass.
    int project_line_search_nested_steps = 0;

    /// Whether smoothing refuses a move that raises the worst incident element's quality. True
    /// is TriWild's and SimWild's behaviour; see optimization::SmoothVertexOptions::quality_veto
    /// for the objective it stops being the right rule for.
    bool smooth_quality_veto = true;
    double w_envelope = 1. - 1e-4; // derived; not read from json

    /// Number and placement of smoothing passes in the shared Wild optimization driver.
    int num_smoothing_passes = 2;
    bool interleaved_smoothing = true;
    int interleaved_smoothing_passes = 1;

    // ---- Coarsening pass -------------------------------------------------
    /**
     * A final pass that removes vertices without letting the max energy rise.
     *
     * The ordinary collapse refuses anything whose resulting cells are worse than the ring
     * they replace, and it judges that on the raw post-collapse geometry -- the worst moment
     * in the operation's life, before smoothing has had any chance to absorb the damage. So a
     * collapse that would be perfectly fine once its neighbourhood relaxes never happens, and
     * the converged mesh carries vertices it does not need.
     *
     * This pass takes the collapse optimistically instead: no quality pre-check, then
     * coarsen_local_smoothing_passes sweeps of smoothing over the coarsen_smooth_ring around
     * the merged vertex, and only then a decision -- keep it if the worst cell in the region
     * touched is no worse than before, undo the whole block otherwise. Because every cell
     * outside that region is untouched, "no worse locally" is exactly "no worse globally".
     */
    bool coarsen_pass = true;
    /**
     * Coarsen as far as the quality guarantee allows, instead of stopping at the target edge
     * length.
     *
     * The pass answers "how few elements can hold this max energy", and left unbounded that is
     * a much more aggressive question than it sounds -- the answer ignores how big the elements
     * become. Measured on tetwild's integration models it takes meshes from 40008 to 3563 cells
     * at unchanged max energy, because a converged mesh is sized by `l` and the adaptive sizing
     * field, not by what the quality target strictly requires, and all of that slack is
     * available once nothing bounds the element size.
     *
     * Turned off, the pass instead stops at the target edge length: an edge already at or past
     * `collapsing_l2` (0.8 * l, the same threshold the ordinary collapse uses) is left alone,
     * because collapsing it only makes its neighbours longer still. The sizing FIELD is
     * deliberately not applied -- that is a local refinement request driven by the optimizer's
     * own history, and honouring it here would leave the pass unable to undo refinement that
     * turned out to be unnecessary. The target length is the user's stated intent; the sizing
     * field is the optimizer's scratch work.
     *
     * On by default: the element count is the thing worth having, and a mesh that meets its
     * quality target with an eighth of the cells is the better answer even though its elements
     * are larger than length_rel nominally asked for. Turn it off to hold the target size.
     */
    bool coarsen_unbounded = true;
    /// Smoothing sweeps over the ring, inside each candidate collapse, before judging it.
    int coarsen_local_smoothing_passes = 2;
    /// Radius smoothed inside the collapse. The lock claims one more ring than this, because
    /// smoothing a vertex reads its one-ring and writes the quality of its incident cells.
    int coarsen_smooth_ring = 2;
    /**
     * Ordinary whole-mesh smoothing passes between coarsening rounds.
     *
     * This is what makes a second round worth running at all -- see coarsen_max_rounds. Set
     * it to 0 and the rounds collapse to one.
     */
    int coarsen_global_smoothing_passes = 1;
    /**
     * Cap on the collapse/smooth alternation. It also stops as soon as a round accepts nothing.
     *
     * A round does NOT exist to finish what the previous one started: within a round the
     * collapse pass already runs to a fixed point (run_localized_to_convergence loops the
     * executor until nothing succeeds, re-offering failures whose neighbourhood changed), so
     * repeating the collapse alone finds nothing. What a round adds is the global smoothing
     * in between, which MOVES that fixed point.
     *
     * Two things stop the inner loop from absorbing it. The dirty-epoch retry re-offers a
     * failed collapse only if one of its endpoints was stamped by a SUCCESSFUL collapse's
     * renewal, so a whole-mesh smoothing pass -- which moves vertices no collapse touched --
     * is invisible to it. And a rejected composite is rolled back in full, its local smoothing
     * included, so rejections never accumulate progress within a round. The global pass is the
     * only geometry improvement that persists and unlocks further collapses.
     *
     * The returns decay fast, because each smoothing pass leaves the mesh closer to relaxed
     * than the last. Measured over the 16 challenging triwild models at five rounds, accepted
     * collapses by round were 68.8% / 21.7% / 7.2% / 1.8% / 0.5%. Every round costs a full
     * collapse_edge_before sweep over all edges plus a smoothing pass, so the default is two:
     * they carry 90.5% of the coarsening between them, and the three that would follow are
     * worth 9.5% for 60% of the pass's budget.
     */
    int coarsen_max_rounds = 2;

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
