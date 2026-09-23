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
 * The optimization phase reuses wmtk::TriOptimizerMesh / wmtk::TetOptimizerMesh, so everything
 * that phase reads -- target edge length, the split/collapse thresholds derived from it,
 * smoothing weights and pass count, the sizing knobs, the debug switch -- comes from
 * wmtk::OptimizerParameters and is not restated here. The bounding box stays here because its
 * type and meaning differ per application.
 *
 * Every key below means the same thing in 2D and 3D; the two pipelines are the same algorithm one
 * dimension apart, and a key that only one of them read would be a difference in the algorithm.
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
    // Turn a non-converged run into a hard error instead of a warning. Off by default: a run that
    // misses the target is still a usable offset, and the warnings name the criterion that failed.
    // Integration tests set it true so a convergence regression fails rather than warns.
    bool throw_on_nonconvergence;
    /// See the spec: false stops the run at the constructed offset, optimize_offset() is not
    /// called and the constructed band is written as the result.
    bool optimize_offset;
    // Half-width of the envelope that contains every tag-region boundary during optimization.
    // Absolute; if < 0, computed from envelope_size_rel (relative to the bbox diagonal).
    double envelope_size;
    double envelope_size_rel;

    // ---- the smooth offset potential ----
    // Support radius of the potential, as a multiple of target_distance. Must be > 1: the offset
    // level set has to lie strictly inside the support, or the vertices on it get no gradient. A
    // band vertex that travels past the support is a hard error, not a silently frozen vertex.
    double offset_dhat_factor;
    /// [2D ONLY] Debugging override for the potential's support radius: >= 0 uses this dhat as an
    /// ABSOLUTE length, in place of both offset_dhat_factor x target_distance and the
    /// constructed-offset floor. Negative (the default) leaves the automatic sizing alone. Only
    /// the 2D construction reads it; the 3D twin ignores it until the port.
    double debug_manual_dhat;
    std::string offset_field; ///< "smooth" (Phi level set) or "euclidean" (exact distance)
    // The accuracy: this fraction of target_distance is both the vertex bar (the remaining Newton
    // step of a front vertex along its move direction, under the default criterion) and the chord
    // resolution threshold (a front edge is refinable while its sag over the level set exceeds
    // it). The run criterion and every placement stop are the same test, so the run converges
    // exactly when every visit stops immediately and no chord is left to resolve.
    double front_conv_rel;
    // Which convergence test gates the run, used identically by the loop's vertex test and the
    // placement stop. F is the vertex's front objective, g its gradient, H its Gauss-Newton
    // Hessian, n its move direction; all four compare against front_conv_rel. See
    // front_vertex_conv_ratio().
    //   "step_size_rel" (the default): the remaining 1-D Newton step, |n.g| / (n^T H n), against
    //     rel x target_distance.
    //   "decrement": the Newton decrement, half of (n.g)^2 / (n^T H n), against rel x F.
    //   "gradient_norm_rel": |n.g| against rel x the reference gradient, measured once on the
    //     band as constructed.
    //   "residual_error": not a stationarity measure at all -- the field's own residual at the
    //     vertex as a length (OffsetPotential::residual_length(), so |d - target_distance| for
    //     the euclidean field and the ENERGY residual for the smooth one), against
    //     rel x target_distance. No objective is built and n does not enter.
    /// gradient_norm_rel | step_size_rel | decrement | residual_error
    std::string front_conv_criterion;
    // The front is placed by a one-dimensional solve along its field normal
    // n = grad Phi / |grad Phi| -- same objective, solver and accept test, restricted to the line
    // x0 + s n -- instead of a free solve. Where a vertex sits along the front carries no offset
    // information, and in the free solve that tangential motion made fronts slide and fold where
    // two of them meet.
    bool front_normal_projection = true;
    bool front_alignment_energy = true; ///< see the spec: needed at pressed seams, biased elsewhere
    /// What a collapse's surviving vertex keeps as its sizing scalar. true (the default): the
    /// smaller of the two, which is the shared engine's rule -- refinement then never relaxes
    /// behind a travelling front. false: the survivor's own.
    bool sizing_collapse_min = true;
    /// Other input regions (no input-complex simplex, no wall contact) deform under smoothing
    /// against their rest shape instead of being envelope-held. See the spec doc.
    bool deform_others = true;
    /// The outer loop's budget in turns. The loop leaves on the front test; this is only the
    /// guard.
    int max_rounds = 40;
    // Points sampled in the interior of each band simplex when measuring the offset's residual;
    // k = 1 is the midpoint, and 0 measures only at band vertices, which is blind to a band whose
    // vertices sit on the level set while its simplices cut across it. 2D samples each band edge
    // at i/(k+1); 3D samples each offset-surface face, k being the density (1, 3, 6, 10 points for
    // k = 1..4). See TopoOffsetTriMesh::offset_edge_samples,
    // TopoOffsetTetMesh::offset_face_samples.
    int offset_residual_samples;
    bool sorted_marching;
    /// See the spec: the marching places each new vertex where d(x) reaches target_distance
    /// along the edge by sphere tracing, midpoint when the trace leaves the edge.
    bool sphere_trace_initialization;
    double sphere_trace_target_rel_tol; ///< |d - target| <= tol x target ends the trace
    /// EXPERIMENTAL. Makes the marching construction all-or-nothing: normally a sphere trace that
    /// leaves its edge falls back to the midpoint for THAT edge alone, so one construction can
    /// mix vertices sitting on the level set with vertices sitting at edge midpoints. With this
    /// on, the march is probed first, and a single untraceable edge sends EVERY edge to its
    /// midpoint. Only sphere_trace_initialization can mix, so this is a no-op when that is off.
    /// See the spec doc, and marching_tris() / marching_tets().
    bool experimental_consistent_construction_split = true;
    std::string output_path; // no extension
    bool save_vtu;

    // Samples per side of the grid the smooth offset potential is written on, beside the result,
    // for the viewer. 0 disables it. The whole domain in 2D, one plane through the box in 3D.
    int phi_grid_resolution;

    int num_threads; // number of threads for parallel execution (smoothing, collapse). 0 = serial
    /// Cap of the shared TriWild/TetWild loop wherever it runs: the pre-optimisation pass and the
    /// frozen-front finishing pass.
    int max_iterations;
    /// Run TriWild/TetWild over the INPUT mesh before the simplicial embedding and the marching,
    /// held only by the per-tag region envelopes, against a sizing field of 1.0 at every vertex.
    /// See pre_optimize_input_mesh() in either mesh.
    bool pre_optimize_input = true;
    /// The operation passes' offset envelope width, as a fraction of target_distance -- the same
    /// tube every turn, rebuilt after every smoothing pass; see rebuild_offset_envelope(). Also
    /// feeds the derived sizing floor (min_edge_length_rel < 0).
    double offset_envelope_rel;

    // l_min from the paper: the shortest edge the sizing field may ask for, given as a multiple of
    // target_distance rather than of the bounding box because that is the scale the offset has;
    // min_edge_length is derived from min_edge_length_rel in init() when negative. A floor on
    // refinement, so raising it makes the result coarser. When not given, it falls back to the
    // offset envelope eps, following TetWild: a surface pinned only to within eps cannot buy
    // fidelity from shorter edges, so this is a runaway rail, not a resolution setting.
    double min_edge_length;
    double min_edge_length_rel;

    // ---- sizing field ----
    // bounds for VertexAttributes::m_sizing_scalar
    double min_sizing_scalar;
    double max_sizing_scalar;
    // gradation cap: neighboring vertices' sizing scalars may differ by at most this factor,
    // enforced by propagating the refinement outward (monotone, only ever lowers a
    // neighbor's scalar). <= 1 disables gradation entirely. Only used by "ring" mode.
    double sizing_gradation;
    /// See the spec: how a lowered sizing scalar spreads to the vertices around it. "ring" =
    /// the base gradation_smooth_sizing, ring by ring at sizing_gradation x per ring;
    /// "distance" = TetWild's adjust_sizing_field ramp, a factor 0.5 at a seed rising linearly
    /// to 1 at distance 1.8 l, applied within that ball only.
    std::string sizing_gradation_mode;
    /// See the spec: the interleaved smoothing of each operation group runs until the front's
    /// Newton-step ratio converges or stalls AND the background's step settles, instead of a
    /// fixed interleaved_smoothing_passes.
    bool adaptive_smoothing;
    int adaptive_smoothing_max_passes; ///< cap on the passes per group
    double adaptive_smoothing_stall_rel; ///< front stalled: max ratio dropped by less than this
    double adaptive_smoothing_step_rel; ///< background settled: max step / (s_v l) at or below
    /// EXPERIMENTAL. See the spec: how a turn resolves the front faces/chords whose sag is over
    /// the tube. "sizing_half" (the default) halves the sizing scalar at their corners,
    /// "sizing_curvature" lowers it to the chord rule's curvature-derived target, and
    /// "split_longest" (3D only) leaves the sizing field alone at 1.0 and force-splits each
    /// simplex's longest edge instead. Replaces the old bool sag_halve_refinement, whose true /
    /// false are now "sizing_half" / "sizing_curvature".
    std::string experimental_refinement_strat;
    /// EXPERIMENTAL, 3D only. See the spec: WHAT MEASURE decides an offset-surface face is
    /// resolved, as a multiple of the bar front_conv_rel x target_distance. "sag" (the only
    /// option today, and the default) is the loop exactly as it has always been: the field's
    /// sagitta at the face centroid against the mean of its three corners. The key exists so the
    /// measure can be swapped without hunting down its call sites -- everything that asks the
    /// question goes through TopoOffsetTetMesh::face_resolution_ratio().
    std::string experimental_resolution_criteria;
    /// EXPERIMENTAL, 3D only. See the spec: the bar for EXPERIMENTAL_resolution_criteria
    /// "normal_deviation", in DEGREES -- the paper's sigma_max. A face is resolved when the angle
    /// between the offset field's normal at its centroid and that normal at each of its three
    /// near-corner samples stays under this. Read by nothing when the criteria is "sag".
    double experimental_max_normal_deviation;
    /// See the spec: true runs one smoothing block (the fixed interleaved count, or the adaptive
    /// smoothing) before the first turn of the single-phase loop.
    bool pre_smooth;
    /// EXPERIMENTAL. See the spec: reject any operation on the offset surface that raises the
    /// local sag -- a collapse whose survivor is left with a worse maximum than the two
    /// endpoints had between them, or a surface flip whose two new faces are worse than the two
    /// old ones. Splits are never rejected. false = the operation passes exactly as they are.
    /// Default true.
    bool experimental_ops_divergence_guard;
    /// DEBUG, 3D only. See the spec: make EXPERIMENTAL_ops_divergence_guard's COLLAPSE test the
    /// pre-2026-09-22 one -- the maximum resolution measure over the two endpoints' offset faces
    /// before against the maximum over the survivor's after, refused on any strict rise -- in
    /// place of today's per-face pairwise test. Diagnostic only, for reproducing the churn the
    /// pairwise test was written to fix. Read by nothing when the guard is off. Default false.
    bool debug_collapse_ring;
    /// EXPERIMENTAL, 3D only (2D has no swap half to the guard). See the spec: how much of the
    /// resolution bar a flip of the offset surface must WIN for the guard to accept it. It gates
    /// the FLIP -- max sag after <= max sag before - this, and the cells under stop_energy, is
    /// the whole rule; one that misses the margin is refused rather than falling back on AMIPS.
    /// Without it the swap pass does not finish, on noise-sized flips that are all monotone.
    double experimental_flip_sag_margin;
    /// EXPERIMENTAL, 3D only. See the spec: true lets the single-phase loop exit on the FIRST
    /// turn that meets the front criterion, the way TetWild's loop stops on its own metric.
    /// false additionally requires that the previous turn lowered no sizing scalar, which is one
    /// turn of hysteresis against the tail's churn. Default true.
    bool experimental_exit_when_criteria_met;
    /// EXPERIMENTAL, 3D only. See the spec: which extra pass, if any, moves the offset-surface
    /// vertices before each smoothing block. "none" (the default) is the loop as it has always
    /// been. "quadrics" runs the error-quadric relocation of Zint et al. 2023 Sec. 5.5, which
    /// redistributes vertices TANGENTIALLY so the 1-D normal solve that follows can put them back
    /// on the level set. "tangential" does the same job with a different energy: the 2-D AMIPS of
    /// the vertex's offset one-ring, projected into the level set's own tangent plane.
    std::string experimental_surface_smoothing_method;
    /// EXPERIMENTAL, 3D only. See the spec: how many passes of
    /// experimental_surface_smoothing_method to run per smoothing block. Ignored when the method
    /// is "none".
    int experimental_surface_smoothing_passes;

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
        optimize_offset = json_params["optimize_offset"];
        envelope_size = json_params["envelope_size"];
        envelope_size_rel = json_params["envelope_size_rel"];
        offset_dhat_factor = json_params["offset_dhat_factor"];
        debug_manual_dhat = json_params["DEBUG_manual_dhat"];
        offset_field = json_params["offset_field"];
        front_conv_rel = json_params["front_conv_rel"];
        front_conv_criterion = json_params["front_conv_criterion"];
        offset_residual_samples = json_params["offset_residual_samples"];

        sorted_marching = json_params["sorted_marching"];
        sphere_trace_initialization = json_params["sphere_trace_initialization"];
        sphere_trace_target_rel_tol = json_params["sphere_trace_target_rel_tol"];
        experimental_consistent_construction_split =
            json_params["EXPERIMENTAL_consistent_construction_split"];
        output_path = json_params["output"];
        save_vtu = json_params["save_vtu"];
        phi_grid_resolution = json_params["phi_grid_resolution"];

        num_threads = json_params["num_threads"];
        max_iterations = json_params["max_iterations"];
        offset_envelope_rel = json_params["offset_envelope_rel"];

        min_edge_length = json_params["min_edge_length"];
        min_edge_length_rel = json_params["min_edge_length_rel"];

        min_sizing_scalar = json_params["min_sizing_scalar"];
        max_sizing_scalar = json_params["max_sizing_scalar"];
        sizing_gradation = json_params["sizing_gradation"];
        sizing_gradation_mode = json_params["sizing_gradation_mode"];
        adaptive_smoothing = json_params["adaptive_smoothing"];
        adaptive_smoothing_max_passes = json_params["adaptive_smoothing_max_passes"];
        adaptive_smoothing_stall_rel = json_params["adaptive_smoothing_stall_rel"];
        adaptive_smoothing_step_rel = json_params["adaptive_smoothing_step_rel"];
        experimental_refinement_strat = json_params["EXPERIMENTAL_refinement_strat"];
        experimental_resolution_criteria = json_params["EXPERIMENTAL_resolution_criteria"];
        experimental_max_normal_deviation = json_params["EXPERIMENTAL_max_normal_deviation"];
        pre_smooth = json_params["pre_smooth"];
        experimental_ops_divergence_guard = json_params["EXPERIMENTAL_ops_divergence_guard"];
        debug_collapse_ring = json_params["DEBUG_collapse_ring"];
        experimental_flip_sag_margin = json_params["EXPERIMENTAL_flip_sag_margin"];
        experimental_exit_when_criteria_met = json_params["EXPERIMENTAL_exit_when_criteria_met"];
        experimental_surface_smoothing_method =
            json_params["EXPERIMENTAL_surface_smoothing_method"];
        experimental_surface_smoothing_passes =
            json_params["EXPERIMENTAL_surface_smoothing_passes"];

        // ---- inherited from wmtk::OptimizerParameters ----
        debug_output = json_params["DEBUG_output"];
        lr = json_params["length_rel"];
        l = json_params["length"];
        stop_energy = json_params["stop_energy"];
        num_smoothing_passes = json_params["num_smoothing_passes"];
        interleaved_smoothing = json_params["interleaved_smoothing"];
        interleaved_smoothing_passes = json_params["interleaved_smoothing_passes"];
        split_high_valence_threshold = json_params["split_high_valence_threshold"];
        // skip_good_regions is deliberately not exposed: it would restrict a smoothing pass to
        // cells still far from stop_energy, but the smoother is what places the offset boundary,
        // so a well-shaped yet badly-placed patch is exactly what must not be skipped.
        // Every key of the coarsening group is copied here, not just the on/off switch: declaring
        // a key in the spec only makes jse inject its default into the json, so a key nothing
        // copies into this struct silently keeps whatever OptimizerParameters holds.
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
        front_normal_projection = json_params["front_normal_projection"];
        front_alignment_energy = json_params["front_alignment_energy"];
        sizing_collapse_min = json_params["sizing_collapse_min"];
        deform_others = json_params["deform_others"];
        max_rounds = json_params["max_rounds"];
        pre_optimize_input = json_params["pre_optimize_input"];
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

        // Not a user knob: a topological offset preserves the topology of the region it wraps, so
        // the shared collapse must always apply the substructure link condition, or a collapse
        // across a thin band pinches the two sides together and the region stops being manifold.
        // Set here because tetwild and simwild leave the flag off.
        preserve_topology = true;

        // Fills diag_l, l/lr and splitting_l2 / collapsing_l2. It also derives eps from epsr,
        // which the offset never reads: its envelope tolerance is m_envelope_eps, set on the mesh.
        init_lengths_from_diagonal((max_ - min_).norm());

        if (target_distance > 0) {
            target_distance_rel = target_distance / diag_l;
        } else {
            target_distance = target_distance_rel * diag_l;
        }

        // An ordinary relative length: it bounds how far a region boundary may drift in space, so
        // the bounding box is the right reference.
        if (envelope_size > 0) {
            envelope_size_rel = envelope_size / diag_l;
        } else {
            envelope_size = envelope_size_rel * diag_l;
        }

        // l_min is relative to the offset distance rather than the bounding box: it is the offset
        // that has to be resolved. See the declaration.
        if (min_edge_length_rel < 0) {
            // The envelope eps as a multiple of target_distance, which is what offset_envelope_rel
            // already is, so there is no conversion left to do.
            min_edge_length_rel = std::max(offset_envelope_rel, 1e-12);
        }
        if (min_edge_length < 0) {
            min_edge_length = min_edge_length_rel * target_distance;
        } else {
            min_edge_length_rel = min_edge_length / std::max(target_distance, 1e-16);
        }
    }
};
} // namespace wmtk::components::topological_offset
