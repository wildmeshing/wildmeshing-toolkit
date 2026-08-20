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
    bool respect_all_topologies;
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
    // Capture that envelope from the input mesh, before the offset truncates the region
    // boundaries it grows through. See TopoOffsetTriMesh::init_region_boundary_envelope_from_input.
    // 2D only; the 3D path never builds this envelope.
    bool region_envelope_from_input;
    // Subdivision floor for conservative offset growth, as a fraction of target_distance: a
    // candidate circle/sphere smaller than this stops being subdivided and is decided outright.
    double relative_ball_threshold;

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
    // Convergence tolerance on the offset, as a fraction of target_distance. The quantity it
    // bounds is the Phi RESIDUAL expressed as a length -- the first-order distance from a band
    // vertex to the level set -- so 0.1 means "every offset vertex is within 10% of the offset
    // distance of where the potential says it belongs".
    double offset_residual_rel;
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
    // g/2. The default is therefore 2 x offset_residual_rel's default, so a run that used to pass
    // the residual bound passes this one on such a field. Away from that case the two are
    // genuinely different tests and neither implies the other.
    //
    // MEASURED OVER THE SURFACE, NOT JUST AT ITS VERTICES. E is a field, so it has a gradient
    // at every point of space, and it is evaluated at interior samples of every band face on the
    // same lattice the residual uses (offset_residual_samples). Both terms gate: a triangle whose
    // corners sit on the level set while its interior chords across it fails this bound, which a
    // vertex-only test reads as converged. Measured on prism at tau = 0.01: the vertex term was
    // under tolerance from round 4, while the in-face term needed four more rounds and was still
    // 10.7x larger at convergence. The two are reported separately because they call for
    // different remedies -- at-vertex wants smoothing, in-face wants refinement.
    double offset_gradient_rel;
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
    int ab_smooth_max_passes; ///< cap on Phase B smoothing passes
    double ab_smooth_tol;
    /// Phase B's convergence criterion: stop when the largest per-vertex placement gradient
    /// falls to this fraction of its value at phase entry. See phase_b_band_gradient_linf().
    double
        ab_smooth_grad_tol_rel; ///< "nothing moves any more", as a fraction of the target length l
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
    // eps = ab_offset_envelope_rel * offset_residual_rel * target_distance, so that product is
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
        respect_all_topologies = json_params["respect_all_topologies"];
        offset_in = json_params["offset_in"];
        offset_out = json_params["offset_out"];
        target_distance = json_params["target_distance"];
        target_distance_rel = json_params["target_distance_rel"];
        throw_on_nonconvergence = json_params["throw_on_nonconvergence"];
        envelope_size = json_params["envelope_size"];
        envelope_size_rel = json_params["envelope_size_rel"];
        region_envelope_from_input = json_params["region_envelope_from_input"];
        relative_ball_threshold = json_params["relative_ball_threshold"];
        offset_dhat_factor = json_params["offset_dhat_factor"];
        offset_field = json_params["offset_field"];
        offset_residual_rel = json_params["offset_residual_rel"];
        offset_gradient_rel = json_params["offset_gradient_rel"];
        offset_residual_samples = json_params["offset_residual_samples"];
        if (relative_ball_threshold < 0.0 || relative_ball_threshold > 1.0) {
            log_and_throw_error(
                "Invalid relative_ball_threshold [{}], must be between 0 and 1.",
                relative_ball_threshold);
        }

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
        ab_smooth_tol = json_params["ab_smooth_tol"];
        ab_smooth_grad_tol_rel = json_params["ab_smooth_grad_tol_rel"];
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
            min_edge_length_rel = std::max(ab_offset_envelope_rel * offset_residual_rel, 1e-12);
        }
        if (min_edge_length < 0) {
            min_edge_length = min_edge_length_rel * target_distance;
        } else {
            min_edge_length_rel = min_edge_length / std::max(target_distance, 1e-16);
        }
    }
};
} // namespace wmtk::components::topological_offset
