#pragma once
#include <wmtk/OptimizerParameters.h>
#include <nlohmann/json.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/expression_parser/Expression.hpp>

using ExpressionPtr = wmtk::components::simwild::expression_parser::ExpressionPtr;

namespace wmtk::components::topological_offset {

/**
 * @brief What the offset needs on top of the parameters every wmtk optimizer shares.
 *
 * The optimization phase reuses wmtk::TetOptimizerMesh, so the quantities that phase reads --
 * the target edge length and the split/collapse thresholds derived from it, the smoothing
 * weights and pass count, the sizing-refinement knobs, the debug switch -- come from
 * wmtk::OptimizerParameters rather than being spelled out again here. The json keys are
 * unchanged where they already matched, and where they did not the base's own name is now used:
 * `length`/`length_rel` feed the base's `l`/`lr`, `DEBUG_output` feeds `debug_output`, and the
 * loop and sizing knobs are spelled exactly as TriWild spells them.
 *
 * The bounding box stays here, as it does in every application: the base deliberately does not
 * own it, because its type and meaning differ per application.
 */
struct Parameters : public wmtk::OptimizerParameters
{
    ExpressionPtr offset_selection;
    std::set<std::string> offset_output_tag;
    std::set<std::string> protected_tags;
    bool offset_in;
    bool offset_out;
    double target_distance;
    double target_distance_rel;
    // Turn a non-converged run into a hard error instead of a warning. Off by default -- a run
    // that misses the target is still a usable offset, and the warnings already name the criterion
    // that failed. Integration tests set it true so a regression in convergence fails the test
    // rather than passing with a warning nobody reads.
    bool throw_on_nonconvergence;
    // Half-width of the envelope that contains every tag-region boundary during optimization.
    // Absolute; if < 0, computed from envelope_size_rel (relative to the bbox diagonal).
    double envelope_size;
    double envelope_size_rel;

    // ---- the smooth offset potential ----
    // Support radius of the potential, as a multiple of target_distance. Must be > 1: the offset
    // level set has to lie strictly inside the support, or the vertices on it get no gradient.
    // 2 by default -- the potential is active out to twice the offset distance, and a band vertex
    // that travels further than that is a hard error rather than a silently frozen vertex.
    double offset_dhat_factor;
    std::string offset_field; ///< "smooth" (Phi level set) or "euclidean" (exact distance)
    /// Only read when debug_output is set: also write the engine's per-pass debug_{N}
    /// frames, not just the per-phase timeline. See the spec doc.
    bool debug_output_per_pass;
    // CONVERGENCE, 3D. The bound on the max over reachable band vertices of the gradient of the
    // offset energy E = (Phi(x) - c)^2 with respect to the vertex position, as a fraction of
    // target_distance.
    //
    // WHY THE GRADIENT AND NOT THE RESIDUAL. The residual is a statement about one particular
    // Phi: it is comparable to target_distance only because residual_length() converts the field
    // value into a length, and every potential has to supply that conversion for the bound to
    // mean the same thing. The gradient is the stationarity condition of the objective Phase B
    // actually minimises, so it is the same statement for ANY Phi -- exact Euclidean, the OGC
    // rule, or ESP -- without the potential having to agree on a length scale first. That is what
    // makes this the criterion to carry into a smooth potential on reentrant geometry.
    //
    // THE SCALE. grad E = 2 (Phi - c) grad Phi, so for a distance-like field (|grad Phi| = 1 near
    // the level set, which is exactly the convex case) the bound |grad E| <= g is |Phi - c| <=
    // g/2. That half is now the ONLY residual scale in the component: offset_residual_tolerance()
    // is g/2 * target_distance, and the Phase A offset envelope is ab_offset_envelope_rel times
    // that. There used to be a separate offset_residual_rel knob supplying both, which meant the
    // envelope was sized off a parameter that stopped gating anything when the criterion moved to
    // the gradient -- measured on prism, Phase A held the surface in a tube of 0.00209 while the
    // configured criterion permitted an error of 0.0837, a factor of 40. One knob, one scale.
    //
    // MEASURED OVER THE SURFACE, NOT JUST AT ITS VERTICES. E is a field, so it has a gradient
    // at every point of space, and it is evaluated at interior samples of every band face on the
    // same lattice the residual uses (offset_residual_samples). Both terms gate: a triangle whose
    // corners sit on the level set while its interior chords across it fails this bound, which a
    // vertex-only test reads as converged. Measured on prism at tau = 0.01: the vertex term was
    // under tolerance from round 4, while the in-face term needed four more rounds and was still
    // 10.7x larger at convergence. The two are reported separately because they call for
    // different remedies -- at-vertex wants smoothing, in-face wants refinement.
    //
    // CONVERGENCE, 2D -- same key, different meaning (see .claude/CLAUDE.md, PARAMETER
    // MEANINGS THAT HAVE MOVED, and the spec doc). The bar is this fraction of a MEASURED
    // reference -- max |2 (Phi - c) grad Phi . n| over the INITIAL offset vertices, n the
    // surface's own Voronoi-length-weighted normal -- and both the run criterion and every
    // Phase B local placement stop compare the FULL gradient norm AT VERTICES against it, one
    // identical test. Edge-interior samples are a chord diagnostic in 2D: reported, never
    // gating.
    double phase_b_conv_rel;
    // WHICH CONVERGENCE CRITERION GATES THE RUN. "gradient" (default): the measured-reference
    // gradient bar above, byte-identical to before this key existed. "dist_and_orient": a geometric
    // criterion in units of target_distance -- see the two keys below and
    // TopoOffsetTriMesh::distance_criterion(). 2D only; 3D reads only "gradient".
    std::string phase_b_conv_criterion; ///< 2D: gradient_norm_rel | step_size_rel | decrement
    // "dist_and_orient" only. Every reachable offset vertex AND every edge-interior sample must lie
    // within this fraction of target_distance of the level set, first order:
    // |Phi - c| / |grad Phi|. Vertices are the placement half, samples the resolution half.
    // "dist_and_orient" only. Largest angle, in degrees, between an offset edge's OUTWARD normal
    // and the field's own outward direction at the edge midpoint. Signed: a folded edge, whose
    // outward normal points into the band by the field's reckoning, fails outright.
    // PHASE B NORMAL-ONLY PLACEMENT (Uday, 2026-08-25): a front vertex moves only along the field's
    // normal at its visit's start. Where a vertex sits along the front carries no information about
    // the offset (a free gauge, redistributed by Phase A's smoother); the 2-D solve's tangential
    // component is the AMIPS step alone, which made fronts slide and fold where two of them meet.
    // Imposed through the front's own energy (a stiff quadratic penalty on tangential
    // displacement, see phase_b_front_energy), so the shared smoother is untouched; the pass stop
    // and the loop's vertex test then measure |grad F . n|, the derivative along the only unknown.
    bool phase_b_normal_only = false;
    bool phase_b_alignment = true;
    /// 2D: false (the default) is the single phase -- TriWild's loop with the front placed inside
    /// its smoothing passes; true is the A/B loop, which is also what 3D always runs.
    bool alternating_opt = false;
    /// 2D single phase: the turn budget. The loop leaves on the front test; this is the guard.
    int single_max_turns = 40;
    // Points sampled in the INTERIOR of each band edge when measuring the offset's residual;
    // k = 1 is the midpoint. 0 falls back to measuring only at band vertices, which is blind to
    // a band whose vertices sit on the level set while its edges cut across it.
    int offset_residual_samples;
    bool sorted_marching;
    std::string output_path; // no extension
    bool save_vtu;
    // Points sampled in the INTERIOR of each offset-surface face when measuring the residual in
    // 3D; offset_residual_samples is the density and the counts are 1, 3, 6, 10 for k = 1..4.
    // (2D samples edges instead, k points at i/(k+1); see TopoOffsetTriMesh::offset_edge_samples
    // and TopoOffsetTetMesh::offset_face_samples.)

    // Samples per side of the grid the smooth offset potential is written on, beside the result,
    // for the viewer. 0 disables it. 2D only.
    int phi_grid_resolution;

    int num_threads; // number of threads for parallel execution (smoothing, collapse). 0 = serial
    // Upper bound on the shared optimization loop, which exits as soon as every convergence
    // criterion is met. TriWild's `max_iterations`, under TriWild's name and default.
    int max_iterations;

    /**
     * The alternating optimization. See TopoOffsetTetMesh::OptPhase for why the two criteria are
     * optimized in turn rather than jointly. 3D only for now; 2D still runs the joint loop.
     */
    int ab_max_rounds; ///< cap on A/B rounds
    bool ab_no_collapse_after_first_round; ///< DIAGNOSTIC: refuse all collapses from round 2
    int ab_phase_a_iterations; ///< iterations of TetWild's loop inside one Phase A
    int ab_smooth_max_passes; ///< cap on Phase B smoothing passes; negative = uncapped (default)
    /// Phase B's GAUSS-SEIDEL pass budget. Each pass gives every eligible vertex exactly ONE
    /// local iteration -- offset vertices one descent step on AMIPS + the offset term, background
    /// vertices one Newton step on their one-ring AMIPS, both line-searched -- and stops early only when
    /// the run's own convergence criterion is met. This replaced solving each vertex to its own
    /// minimum, which let a vertex race to a fixed point its neighbours had not seen yet; where
    /// two offset fronts approach, that is what crushed the elements between them.
    int ab_phase_b_iterations;
    double ab_smooth_tol;
    /// Phase B's INTERIOR (background AMIPS) per-vertex Newton tolerance: polysolve's
    /// rel_grad_norm_tol, the fraction of the visit's own entry gradient the solve stops at.
    /// In 2D the OFFSET placement no longer reads this -- its descent stops on the run's own
    /// bar, offset_gradient_tolerance(), so phase_b_conv_rel governs the local
    /// solves and the global criterion alike (an entry-relative rule with no absolute floor
    /// limit-cycled; see smooth_offset_vertex_backtracking()). 3D still reads it for BOTH of
    /// its Phase B solves -- see .claude/CLAUDE.md, PARAMETER MEANINGS THAT HAVE MOVED.
    double ab_vertex_grad_tol_rel;
    /// Run TriWild over the INPUT mesh before the simplicial embedding and the marching, held
    /// only by the per-tag region envelopes. 2D only; see TopoOffsetTriMesh::pre_optimize_input_mesh().
    bool pre_optimize_input = false;
    /// See the spec: which sizing field pre_optimize_input runs against. false = seed
    /// target_distance on the input-complex boundary; true = seed every vertex from its own
    /// incident edge lengths, so target_distance never enters the field.
    bool pre_optimize_sizing_from_edges = false;
    /// Phase A's offset envelope width, in Phi tolerances -- the same tube every round; see
    /// rebuild_offset_envelope(). Also feeds the derived sizing floor (min_edge_length_rel < 0).
    double ab_offset_envelope_rel;

    // l_min from the paper: the shortest edge the sizing field may ask for. Tied to the OFFSET
    // DISTANCE rather than to the bounding box, because that is the scale the offset actually
    // has -- so it is given relatively, as a multiple of target_distance, and min_edge_length is
    // derived from min_edge_length_rel in init() when negative. This is a floor on refinement,
    // so raising it makes the result COARSER (paper Fig. 18).
    //
    // TETWILD'S FLOOR, IN THE OFFSET'S UNITS, when not given (min_edge_length_rel < 0). The
    // paper caps the sizing field below by the envelope epsilon ("to prevent unnecessary
    // over-refinement in problematic regions", Sec 3.2): the surface is only pinned to within
    // eps, so edges shorter than eps cannot buy fidelity. The offset's envelope is Phase A's,
    // eps = ab_offset_envelope_rel * target_distance, so that
    // product is
    // the derived floor. It is a pure runaway rail, well below the ~delta*sqrt(8*tau) chord any
    // tolerance tau actually needs -- refinement stops at "cannot help" rather than at a fixed
    // resolution. This replaced a fixed 2*sin(15 deg) inherited from the deleted
    // normal-deviation criterion, which encoded tau ~ 3.3% forever regardless of the
    // configured tolerance and made anything tighter unreachable by refinement.
    double min_edge_length;
    double min_edge_length_rel;

    // ---- sizing field, see TopoOffsetTetMesh::update_sizing_field() ----
    // bounds for VertexAttributes::m_sizing_scalar
    double min_sizing_scalar;
    double max_sizing_scalar;
    // mean ratio metric strictly below this is "bad" (refine); strictly above is "good"
    // (coarsen), matching the reference's compute_target_edge_length()
    double sizing_mrm_threshold;
    // gradation cap: neighboring vertices' sizing scalars may differ by at most this factor,
    // enforced by propagating the refinement outward (monotone, only ever lowers a
    // neighbor's scalar). <= 1 disables gradation entirely.
    double sizing_gradation;

    VectorXd box_min;
    VectorXd box_max;

    Parameters() = default;

    Parameters(const nlohmann::json& json_params)
    {
        for (const std::string& tag : json_params["offset_output_tags"]) {
            if (tag == "ambient") {
                logger().warn(
                    "'ambient' tag cannot be given explicitly to offset_output_tags, ignoring. To "
                    "set offset to 'ambient', pass offset_output_tags=[].");
                continue;
            }
            offset_output_tag.insert(tag);
        }
        for (const std::string& tag : json_params["protected_tags"]) {
            if (tag == "ambient") {
                logger().warn("'ambient' tag cannot be protected, ignoring.");
                continue;
            }
            protected_tags.insert(tag);
        }
        offset_in = json_params["offset_in"];
        offset_out = json_params["offset_out"];
        target_distance = json_params["target_distance"];
        target_distance_rel = json_params["target_distance_rel"];
        throw_on_nonconvergence = json_params["throw_on_nonconvergence"];
        envelope_size = json_params["envelope_size"];
        envelope_size_rel = json_params["envelope_size_rel"];
        offset_dhat_factor = json_params["offset_dhat_factor"];
        offset_field = json_params["offset_field"];
        phase_b_conv_rel = json_params["phase_b_conv_rel"];
        phase_b_conv_criterion = json_params["phase_b_conv_criterion"];
        offset_residual_samples = json_params["offset_residual_samples"];

        sorted_marching = json_params["sorted_marching"];
        output_path = json_params["output"];
        save_vtu = json_params["save_vtu"];
        phi_grid_resolution = json_params["phi_grid_resolution"];

        num_threads = json_params["num_threads"];
        max_iterations = json_params["max_iterations"];
        ab_max_rounds = json_params["ab_max_rounds"];
        ab_no_collapse_after_first_round = json_params["ab_no_collapse_after_first_round"];
        ab_phase_a_iterations = json_params["ab_phase_a_iterations"];
        ab_smooth_max_passes = json_params["ab_smooth_max_passes"];
        ab_phase_b_iterations = json_params["ab_phase_b_iterations"];
        ab_smooth_tol = json_params["ab_smooth_tol"];
        ab_vertex_grad_tol_rel = json_params["ab_vertex_grad_tol_rel"];
        ab_offset_envelope_rel = json_params["ab_offset_envelope_rel"];

        min_edge_length = json_params["min_edge_length"];
        min_edge_length_rel = json_params["min_edge_length_rel"];

        min_sizing_scalar = json_params["min_sizing_scalar"];
        max_sizing_scalar = json_params["max_sizing_scalar"];
        sizing_mrm_threshold = json_params["sizing_mrm_threshold"];
        sizing_gradation = json_params["sizing_gradation"];

        // ---- inherited from wmtk::OptimizerParameters ----
        debug_output = json_params["DEBUG_output"];
        debug_output_per_pass = json_params["DEBUG_output_per_pass"];
        lr = json_params["length_rel"];
        l = json_params["length"];
        stop_energy = json_params["stop_energy"];
        num_smoothing_passes = json_params["num_smoothing_passes"];
        interleaved_smoothing = json_params["interleaved_smoothing"];
        interleaved_smoothing_passes = json_params["interleaved_smoothing_passes"];
        split_high_valence_threshold = json_params["split_high_valence_threshold"];
        // skip_good_regions is deliberately NOT exposed. It restricts a smoothing pass to the
        // vertices of cells that are still far from stop_energy, and the offset needs every
        // vertex smoothed every pass: the offset boundary is placed BY the smoother, and a
        // well-shaped but badly-placed patch is exactly what the filter would skip. Left at
        // OptimizerParameters' `false`.
        // The whole coarsening group, not just the on/off switch. Declaring a key in the spec
        // is only half of making it settable: jse injects the default into the json, and if
        // nothing copies it into the struct the value in force is whatever
        // OptimizerParameters happens to hold. Measured the hard way -- setting
        // coarsen_smooth_ring produced bit-identical output because it was never read.
        coarsen_pass = json_params["coarsen_pass"];
        coarsen_unbounded = json_params["coarsen_unbounded"];
        coarsen_local_smoothing_passes = json_params["coarsen_local_smoothing_passes"];
        coarsen_smooth_ring = json_params["coarsen_smooth_ring"];
        coarsen_global_smoothing_passes = json_params["coarsen_global_smoothing_passes"];
        coarsen_max_rounds = json_params["coarsen_max_rounds"];
        stuck_refine_stall_eps = json_params["stuck_refine_stall_eps"];
        stuck_refine_cooldown = json_params["stuck_refine_cooldown"];
        stuck_refine_num_worst = json_params["stuck_refine_num_worst"];
        stuck_refine_rings = json_params["stuck_refine_rings"];
        stuck_refine_factor = json_params["stuck_refine_factor"];
        stuck_refine_min_scalar = json_params["stuck_refine_min_scalar"];
        stuck_refine_gradation = json_params["stuck_refine_gradation"];
        stuck_refine_force_split = json_params["stuck_refine_force_split"];
        phase_b_normal_only = json_params["phase_b_normal_only"];
        phase_b_alignment = json_params["phase_b_alignment"];
        alternating_opt = json_params["alternating_opt"];
        single_max_turns = json_params["single_max_turns"];
        pre_optimize_input = json_params["pre_optimize_input"];
        pre_optimize_sizing_from_edges = json_params["pre_optimize_sizing_from_edges"];
        w_amips = json_params["w_amips"];
        smoothing_mode = json_params["smoothing_mode"];
        project_line_search_steps = json_params["project_line_search_steps"];
        project_line_search_nested_steps = json_params["project_line_search_nested_steps"];
        smooth_quality_veto = json_params["smooth_quality_veto"];
        w_envelope = 1. - w_amips;
        perform_sanity_checks = json_params["perform_sanity_checks"];
    }

    void init(const VectorXd& min_, const VectorXd& max_)
    {
        box_min = min_;
        box_max = max_;

        // Not a user knob. A TOPOLOGICAL offset is defined by preserving the topology of the
        // region it wraps, so the shared collapse must always apply the substructure link
        // condition -- without it a collapse across a thin offset band pinches the two sides
        // together and the region stops being manifold. The offset's own collapse applied this
        // unconditionally before it moved onto the shared engine, where it is gated on this
        // flag; tetwild and simwild leave the flag off, which is why it is set here and not
        // changed in wmtk.
        preserve_topology = true;

        // Fills diag_l, l/lr and splitting_l2 / collapsing_l2 -- the same 16/9 and 16/25
        // factors this used to spell out itself. It also derives eps from epsr, which the
        // offset never reads: its envelope tolerance is m_envelope_eps, set on the mesh.
        init_lengths_from_diagonal((max_ - min_).norm());

        if (target_distance > 0) {
            target_distance_rel = target_distance / diag_l;
        } else {
            target_distance = target_distance_rel * diag_l;
        }

        // An ordinary relative length, unlike convergence_target: it bounds how far a region
        // boundary may drift in space, so the bounding box is the right reference.
        if (envelope_size > 0) {
            envelope_size_rel = envelope_size / diag_l;
        } else {
            envelope_size = envelope_size_rel * diag_l;
        }

        // l_min, relative to the OFFSET DISTANCE rather than the bounding box: it is the offset
        // that has to be resolved, and its scale is delta. See the declaration for why this is
        // no longer derived from an angle.
        if (min_edge_length_rel < 0) {
            // THE ENVELOPE EPS, AS A MULTIPLE OF target_distance -- which is now exactly what
            // ab_offset_envelope_rel is, so no conversion is left to do. The old expression,
            // ab_offset_envelope_rel * 0.5 * phase_b_conv_rel, was that same eps
            // back when the tube was a fraction of the residual tolerance; at the old defaults
            // (0.25, 0.2) it came to 0.025, which is the new default itself, so this is
            // unchanged in value and only stops restating a definition that has moved.
            min_edge_length_rel = std::max(ab_offset_envelope_rel, 1e-12);
        }
        if (min_edge_length < 0) {
            min_edge_length = min_edge_length_rel * target_distance;
        } else {
            min_edge_length_rel = min_edge_length / std::max(target_distance, 1e-16);
        }
    }
};
} // namespace wmtk::components::topological_offset
