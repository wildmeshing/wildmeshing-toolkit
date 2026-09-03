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
    // The accuracy: this fraction of target_distance is both the vertex bar (the remaining Newton
    // step of a front vertex along its move direction, under the default criterion) and the chord
    // resolution threshold (a front edge is refinable while its sag over the level set exceeds
    // it). The run criterion and every placement stop are the same test, so the run converges
    // exactly when every visit stops immediately and no chord is left to resolve.
    double front_conv_rel;
    // Which convergence test gates the run, used identically by the loop's vertex test and the
    // placement stop. F is the vertex's front objective, g its gradient, H its Gauss-Newton
    // Hessian, n its move direction; all three compare against front_conv_rel. See
    // front_vertex_conv_ratio().
    //   "step_size_rel" (the default): the remaining 1-D Newton step, |n.g| / (n^T H n), against
    //     rel x target_distance.
    //   "decrement": the Newton decrement, half of (n.g)^2 / (n^T H n), against rel x F.
    //   "gradient_norm_rel": |n.g| against rel x the reference gradient, measured once on the
    //     band as constructed.
    std::string front_conv_criterion; ///< gradient_norm_rel | step_size_rel | decrement
    // The front is placed by a one-dimensional solve along its field normal
    // n = grad Phi / |grad Phi| -- same objective, solver and accept test, restricted to the line
    // x0 + s n -- instead of a free solve. Where a vertex sits along the front carries no offset
    // information, and in the free solve that tangential motion made fronts slide and fold where
    // two of them meet.
    bool front_normal_projection = true;
    bool front_alignment_energy = true; ///< see the spec: needed at pressed seams, biased elsewhere
    /// What a collapse's surviving vertex keeps as its sizing scalar. false (the default): its
    /// own. true: the smaller of the two, which is the shared engine's rule -- refinement then
    /// never relaxes behind a travelling front.
    bool sizing_collapse_min = false;
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
    /// held only by the per-tag region envelopes. See pre_optimize_input_mesh() in either mesh.
    bool pre_optimize_input = true;
    /// See the spec: which sizing field pre_optimize_input runs against. false = seed
    /// target_distance on the input-complex boundary; true = seed every vertex from its own
    /// incident edge lengths, so target_distance never enters the field.
    bool pre_optimize_sizing_from_edges = false;
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
        offset_envelope_rel = json_params["offset_envelope_rel"];

        min_edge_length = json_params["min_edge_length"];
        min_edge_length_rel = json_params["min_edge_length_rel"];

        min_sizing_scalar = json_params["min_sizing_scalar"];
        max_sizing_scalar = json_params["max_sizing_scalar"];
        sizing_gradation = json_params["sizing_gradation"];

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
