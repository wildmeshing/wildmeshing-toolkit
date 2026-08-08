#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::triwild {
struct Parameters
{
    double epsr = 1e-3; // relative error bound (wrt diagonal)
    double eps = -1.; // absolute error bound
    double lr = 5e-2; // target edge length (relative)
    double l = -1.;
    double l_min = -1;
    double diag_l = -1.;
    Vector2d box_min = Vector2d::Zero();
    Vector2d box_max = Vector2d::Ones();
    bool preserve_topology = false;

    /**
     * Keep the curve network's 0-dimensional features -- open polyline endpoints and
     * junctions -- within eps of where the arrangement put them.
     *
     * Without it the collapse pass deletes open polylines outright: a polyline erodes into
     * its own tip until one segment is left, and that segment has a feature at both ends,
     * which nothing else refuses. Off only for A/B against the old behaviour.
     */
    bool preserve_feature_points = true;
    std::string output_path;

    double splitting_l2 = -1.; // the lower bound length (squared) for edge split
    double collapsing_l2 =
        std::numeric_limits<double>::max(); // the upper bound length (squared) for edge collapse

    double stop_energy = 20;

    bool debug_output = false;
    bool perform_sanity_checks = false;

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
     * Relative weight of the AMIPS (quality) term against the envelope (stay-on-curve) term
     * during smoothing. w_envelope is derived as 1 - w_amips in the TriWildMesh constructor,
     * so the small default means the envelope dominates and AMIPS acts as a light quality
     * preference. Matches tetwild and simwild.
     */
    double w_amips = 1e-4;
    double w_envelope = 1. - 1e-4; // derived; not read from json

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

    // ---- Stuck-element sizing refinement --------------------------------
    // Same names, defaults and meaning as tetwild/simwild -- see the specs.
    //
    // Trigger threshold: fire when the last iteration's improvement is small compared
    // with the distance the max energy still has to cover, i.e. refine when
    //     (prev_max - max) <= stall_eps * (max - stop_energy).
    // Equivalently: refine unless the mesh is on course to reach the target within
    // about 1/stall_eps more iterations. 0 => only when it does not improve at all.
    //
    // The denominator is the remaining distance, not prev_max, and that is the whole
    // point. Measured against prev_max, a mesh grinding down at a steady 1.3% per
    // iteration from 27 toward a target of 5 clears a 1% bar every single iteration and
    // so never looks stalled -- even though at that rate it needs ~130 more iterations
    // and the operations have in fact deadlocked. The escape hatch then never fires,
    // which is exactly the regime it exists for.
    double stuck_refine_stall_eps = 0.1;
    // Cooldown: after a refinement, skip this many improvement iterations before
    // refining again, so the operations get full passes to act on the new sizing
    // field before more refinement is added. 0 => may refine every iteration.
    int stuck_refine_cooldown = 1;
    // Number of worst triangles (by energy) whose neighborhoods are refined.
    // 0 => every triangle above the filter energy, max(max_energy / 100, stop_energy).
    // Beware in 2D: the AMIPS2D energy of an equilateral triangle is 2, so a stop_energy
    // close to that makes the filter catch nearly the whole mesh, and with force_split on
    // that means thousands of gate-bypassing splits per stall.
    int stuck_refine_num_worst = 0;
    // Graph rings around each worst triangle's vertices included in the refinement.
    int stuck_refine_rings = 0;
    // Multiplicative reduction of m_sizing_scalar per refinement (0.5 => /2).
    double stuck_refine_factor = 0.5;
    // Lower bound on m_sizing_scalar.
    double stuck_refine_min_scalar = 1e-3;
    // Gradation cap for the monotone sizing smoothing: neighboring sizings may
    // differ by at most this factor. The smoothing only ever *lowers* sizings
    // (spreads refinement outward), never raises the refined values, avoiding
    // sharp resolution jumps that make operations ill-conditioned.
    double stuck_refine_gradation = 2.0;
    // Force-split: when the max energy stalls, split each worst triangle's longest edge
    // once, bypassing the split length gate. This unsticks a sliver whose edges are
    // too short to be split-eligible, WITHOUT touching the sizing field.
    bool stuck_refine_force_split = true;
    // When a split's rounded (double) midpoint would invert an incident triangle, the
    // split is normally rejected. If this is on, splits of edges that belong to the
    // current worst-triangle set instead fall back to the EXACT rational midpoint
    // (never inverts) and keep the new vertex un-rounded, so the worst region can
    // still be refined.

    // ---- Skip good regions ----------------------------------------------
    // Only smooth vertices incident to a triangle whose energy is >=
    // skip_good_regions_margin * stop_energy. Smoothing a vertex surrounded by
    // good triangles does nothing, so skipping it is free. Only smoothing is
    // gated: gating the topology/sizing ops (split/collapse/swap) starves the
    // optimizer and blows up the element count, so those always run over the
    // whole mesh.
    // Off: gating on element quality also skips SURFACE vertices, whose smoothing is driven
    // almost entirely by the envelope term (w_amips 1e-4), not by the quality of the elements
    // around them. On 122839 the filtered run exhausted 60 iterations at max energy 21.03
    // while the unfiltered one converged to 19.9998 in 54 -- and did so in LESS wall time
    // (875s vs 947s), because needing fewer iterations more than paid for the extra vertices
    // per pass.
    bool skip_good_regions = false;
    // Safety margin on the "active" threshold: a triangle is active when its
    // energy is >= this fraction of stop_energy, so vertices near triangles
    // sitting just below the target are still smoothed.
    double skip_good_regions_margin = 0.9;

    Parameters() = default;

    /**
     * @brief Read every optimizer knob out of the (defaults-injected) JSON.
     *
     * Mirrors simwild's Parameters(json) so the three applications stay in step.
     */
    Parameters(const nlohmann::json& json_params)
    {
        output_path = json_params["output"];

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
        diag_l = (box_max - box_min).norm();
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
