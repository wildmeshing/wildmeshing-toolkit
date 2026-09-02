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
 * The optimization phase reuses wmtk::TetOptimizerMesh, so everything that phase reads -- target
 * edge length, the split/collapse thresholds derived from it, smoothing weights and pass count,
 * the sizing knobs, the debug switch -- comes from wmtk::OptimizerParameters and is not restated
 * here. The bounding box stays here because its type and meaning differ per application.
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
    // Half-width of the envelope that contains every tag-region boundary during optimization.
    // Absolute; if < 0, computed from envelope_size_rel (relative to the bbox diagonal).
    double envelope_size;
    double envelope_size_rel;

    // ---- the smooth offset potential ----
    // Support radius of the potential, as a multiple of target_distance. Must be > 1: the offset
    // level set has to lie strictly inside the support, or the vertices on it get no gradient. A
    // band vertex that travels past the support is a hard error, not a silently frozen vertex.
    double offset_dhat_factor;
    std::string offset_field; ///< "smooth" (Phi level set) or "euclidean" (exact distance)
    /// Only read when debug_output is set: also write the engine's per-pass debug_{N}
    /// frames, not just the per-phase timeline. See the spec doc.
    bool debug_output_per_pass;
    // Convergence, 3D: the bound on the gradient of the offset energy E = (Phi(x) - c)^2 over
    // reachable band vertices, as a fraction of target_distance. The gradient rather than the
    // residual, because it is the stationarity condition of what Phase B minimises and so says the
    // same thing for any Phi, with no length-scale conversion. It is the component's only residual
    // scale: offset_residual_tolerance() is half of it times target_distance, and Phase A's offset
    // envelope is offset_envelope_rel times that. It gates at band vertices and at interior samples
    // of every band face alike -- a face whose corners sit on the level set can still cut across it
    // -- and the two are reported separately: at-vertex wants smoothing, in-face wants refinement.
    //
    // Convergence, 2D -- same key, deliberately different meaning: this fraction of a measured
    // reference, tested as the full gradient norm at vertices. The run criterion and every Phase B
    // placement stop are that one identical test, so the run converges exactly when every visit
    // stops immediately. Edge-interior samples are a reported diagnostic here and never gate.
    double front_conv_rel;
    // Which convergence test gates the run, used identically by the loop's vertex test and Phase
    // B's pass stop. F is the vertex's Phase B objective, g its gradient, H its Gauss-Newton
    // Hessian, n its move direction; all three compare against front_conv_rel. 2D only -- 3D never
    // reads this key and always uses the gradient bound above. See front_vertex_conv_ratio().
    //   "step_size_rel" (the default): the remaining 1-D Newton step, |n.g| / (n^T H n), against
    //     rel x target_distance.
    //   "decrement": the Newton decrement, half of (n.g)^2 / (n^T H n), against rel x F.
    //   "gradient_norm_rel": |n.g| against rel x the reference gradient, measured once on the
    //     band as constructed.
    std::string front_conv_criterion; ///< 2D: gradient_norm_rel | step_size_rel | decrement
    // Phase B places a front vertex by a one-dimensional solve along its field normal
    // n = grad Phi / |grad Phi| -- same objective, solver and accept test, restricted to the line
    // x0 + s n -- instead of a free 2-D solve. Where a vertex sits along the front carries no
    // offset information, and in the free solve that tangential motion made fronts slide and fold
    // where two of them meet.
    bool front_normal_projection = true;
    bool front_alignment_energy = true; ///< see the spec: needed at pressed seams, biased elsewhere
    /// What a collapse's surviving vertex keeps as its sizing scalar. false (the default): its
    /// own. true: the smaller of the two, which is the shared engine's rule -- refinement then
    /// never relaxes behind a travelling front.
    bool sizing_collapse_min = false;
    /// 2D. Other input regions (no input-complex face, no wall contact) deform under smoothing
    /// against their rest shape instead of being envelope-held. See the spec doc.
    bool deform_others = true;
    /// The outer loop's budget: turns of the 2D single phase, A/B rounds in 3D. The loop leaves on
    /// the front test; this is only the guard.
    int max_rounds = 40;
    // Points sampled in the interior of each band simplex when measuring the offset's residual;
    // k = 1 is the midpoint, and 0 measures only at band vertices, which is blind to a band whose
    // vertices sit on the level set while its simplices cut across it. 2D samples each band edge
    // at i/(k+1); 3D samples each offset-surface face, k being the density (1, 3, 6, 10 points for
    // k = 1..4). See TopoOffsetTriMesh::offset_edge_samples,
    // TopoOffsetTetMesh::offset_face_samples.
    int offset_residual_samples;
    bool sorted_marching;
    std::string output_path; // no extension
    bool save_vtu;

    // Samples per side of the grid the smooth offset potential is written on, beside the result,
    // for the viewer. 0 disables it. 2D only.
    int phi_grid_resolution;

    int num_threads; // number of threads for parallel execution (smoothing, collapse). 0 = serial
    /// Cap of the shared TriWild/TetWild loop wherever it runs: the pre-optimisation pass, one
    /// Phase A, the frozen-front finishing pass.
    int max_iterations;
    int ab_smooth_max_passes; ///< cap on Phase B smoothing passes; negative = uncapped (default)
    double ab_smooth_tol;
    /// Phase B's interior (background AMIPS) per-vertex Newton tolerance: polysolve's
    /// rel_grad_norm_tol, the fraction of the visit's own entry gradient the solve stops at. The
    /// 2D front placement does not read it -- an entry-relative rule with no absolute floor
    /// limit-cycles, so that descent stops on offset_gradient_tolerance() and front_conv_rel
    /// governs local solves and the global criterion alike. 3D reads it for both Phase B solves.
    double vertex_grad_tol_rel;
    /// Run TriWild over the INPUT mesh before the simplicial embedding and the marching, held
    /// only by the per-tag region envelopes. 2D only; see TopoOffsetTriMesh::pre_optimize_input_mesh().
    bool pre_optimize_input = true;
    /// See the spec: which sizing field pre_optimize_input runs against. false = seed
    /// target_distance on the input-complex boundary; true = seed every vertex from its own
    /// incident edge lengths, so target_distance never enters the field.
    bool pre_optimize_sizing_from_edges = false;
    /// Phase A's offset envelope width, in Phi tolerances -- the same tube every round; see
    /// rebuild_offset_envelope(). Also feeds the derived sizing floor (min_edge_length_rel < 0).
    double offset_envelope_rel;

    // l_min from the paper: the shortest edge the sizing field may ask for, given as a multiple of
    // target_distance rather than of the bounding box because that is the scale the offset has;
    // min_edge_length is derived from min_edge_length_rel in init() when negative. A floor on
    // refinement, so raising it makes the result coarser. When not given, it falls back to the
    // Phase A envelope eps, following TetWild: a surface pinned only to within eps cannot buy
    // fidelity from shorter edges, so this is a runaway rail, not a resolution setting.
    double min_edge_length;
    double min_edge_length_rel;

    // ---- sizing field ----
    // bounds for VertexAttributes::m_sizing_scalar
    double min_sizing_scalar;
    double max_sizing_scalar;
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
        front_conv_rel = json_params["front_conv_rel"];
        front_conv_criterion = json_params["front_conv_criterion"];
        offset_residual_samples = json_params["offset_residual_samples"];

        sorted_marching = json_params["sorted_marching"];
        output_path = json_params["output"];
        save_vtu = json_params["save_vtu"];
        phi_grid_resolution = json_params["phi_grid_resolution"];

        num_threads = json_params["num_threads"];
        max_iterations = json_params["max_iterations"];
        ab_smooth_max_passes = json_params["ab_smooth_max_passes"];
        ab_smooth_tol = json_params["ab_smooth_tol"];
        vertex_grad_tol_rel = json_params["vertex_grad_tol_rel"];
        offset_envelope_rel = json_params["offset_envelope_rel"];

        min_edge_length = json_params["min_edge_length"];
        min_edge_length_rel = json_params["min_edge_length_rel"];

        min_sizing_scalar = json_params["min_sizing_scalar"];
        max_sizing_scalar = json_params["max_sizing_scalar"];
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
