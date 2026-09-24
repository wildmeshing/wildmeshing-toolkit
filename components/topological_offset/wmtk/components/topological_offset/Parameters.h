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
    // ---- the two convergence epsilons (was the single front_conv_rel) ----
    // Both are ABSOLUTE LENGTHS, resolved in init(); if < 0 each is computed from its _rel twin,
    // which is a fraction of the BOUNDING BOX DIAGONAL -- the same absolute/relative pair as
    // envelope_size / envelope_size_rel, and deliberately NOT a fraction of target_distance any
    // more, so changing the offset distance no longer silently changes the accuracy. init()
    // refuses either one above target_distance: an epsilon coarser than the offset it measures
    // cannot decide anything.
    //
    // THE ONE BAR. 3D measures a single quantity everywhere -- over a face's stencil, the RMS of
    // the field's relative error (Phi - c)/c, expressed as a length -- and compares it against
    // this. A vertex is placed when that same measure at the vertex alone is within it, which is
    // the order-0 stencil, so the vertex test and the face test are one test at two sample
    // counts. Replaces vertex_conv / sag_conv, which split the two apart 2026-09-23.
    double front_conv;
    double front_conv_rel;

    /// The convergence epsilon as a FRACTION of target_distance, which is the form the
    /// dimensionless relative error (Phi - c)/c is compared against. A mean of squared relative
    /// errors is below front_conv_frac()^2 exactly when the same mean taken in lengths is below
    /// front_conv^2 -- the two differ by target_distance^2 on both sides -- so which form the
    /// code uses is a matter of where the division sits, not of what is being asked. 2D's
    /// fraction-valued criteria ('decrement', 'gradient_norm_rel') take it for the same reason.
    double front_conv_frac() const { return front_conv / std::max(target_distance, 1e-16); }
    // 2D ONLY since the 3D criteria were unified. Which convergence test gates a 2D run's vertex
    // placement. F is the vertex's front objective, g its gradient, H its Gauss-Newton Hessian,
    // n its move direction; all four compare against front_conv. See front_vertex_conv_ratio().
    //   "step_size_rel" (the default): the remaining 1-D Newton step, |n.g| / (n^T H n), against
    //     front_conv.
    //   "decrement": the Newton decrement, half of (n.g)^2 / (n^T H n), against
    //     front_conv_frac() x F.
    //   "gradient_norm_rel": |n.g| against front_conv_frac() x the reference gradient, measured
    //     once on the band as constructed.
    //   "residual_error": not a stationarity measure at all -- the field's own residual at the
    //     vertex as a length (OffsetPotential::residual_length(), so |d - target_distance| for
    //     the euclidean field and the ENERGY residual for the smooth one), against front_conv.
    //     No objective is built and n does not enter.
    //
    // 3D DOES NOT READ THIS. Its one measure is the stencil RMS of the relative error, which is
    // not a stationarity test and has no variants -- see TopoOffsetTetMesh::face_conv_ratio().
    /// gradient_norm_rel | step_size_rel | decrement | residual_error [2D ONLY]
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
    // Sampling density of THE measure -- in 3D both the criterion's and the energy's -- and of
    // the residual diagnostics that share the lattice. Order k puts these points on a face:
    //   0 -> 3, the CORNERS alone, so the measure is exactly the three vertices' placement error;
    //   k >= 1 -> the vertices of the triangle subdivided k-1 times by 4-way midpoint refinement
    //   plus the centroid of each of its 4^(k-1) sub-triangles, i.e. 4, 10, 31, 109, ...
    // The corners are IN the stencil, unlike the strictly interior lattice this replaces, because
    // the quantity measured is a distance to the level set rather than an interpolation error and
    // so is not identically zero there. Raising it costs a Phi value and gradient per sample, in
    // the ops guard's hot path as well as in energy_criterion() and every smoothing solve.
    // 2D reads this key for its diagnostics only; its chord test is still the MIDPOINT.
    // See TopoOffsetTetMesh::for_each_face_sample, TopoOffsetTriMesh::offset_edge_samples.
    int stencil_order;
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
    /// EXPERIMENTAL, 3D only. Drops the placement gate on refinement. Normally a face over the
    /// bar is handed to the halving only when all THREE of its corners are already placed, so a
    /// front that is still travelling cannot refine -- the safeguard that stops refinement from
    /// chasing a moving front. With this on, EVERY face over the bar is refined, placed or not,
    /// so the sizing scalar is halved at every vertex of every unresolved face. The floor and
    /// the once-per-vertex-per-turn rule are unchanged, and so is the exit test: a face is
    /// refinable only while the halving can still lower a target, and `n_faces_over_placed` /
    /// `max_face_placed` still report the PLACED subset alone.
    ///
    /// Why it exists: under one unified measure the two halves can deadlock. A face chording a
    /// feature of radius delta puts its centroid far inside the level set, and that sample's
    /// pull cancels the corners' own placement pull almost exactly, so the corners never place;
    /// refinement, which is the only thing that would shorten the chord and remove the sag, is
    /// gated on exactly those corners being placed. Measured on the deliverable cube at
    /// target_distance_rel 1e-2 / front_conv_rel 1e-4: 98% cancellation along the normal, the
    /// 1-D Newton step 1-2% of the move needed, and 600+ faces over the bar with ZERO refinable
    /// for 40 turns.
    bool experimental_aggresive_refine = false;
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
    // The operation passes' offset envelope half-width: the leash the front is kept inside while
    // the operation passes run, the same tube every turn, rebuilt after every smoothing pass; see
    // rebuild_offset_envelope(). Absolute-or-relative exactly as envelope_size / envelope_size_rel
    // and against the same reference, the BOUNDING BOX DIAGONAL -- it is a distance in space, and
    // tying it to target_distance made every change of the offset distance a silent change of the
    // leash as well. Also feeds the derived sizing floor (min_edge_length_rel < 0).
    double offset_envelope; ///< absolute; < 0 means use offset_envelope_rel
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
    /// See the spec: true runs one smoothing block (the fixed interleaved count, or the adaptive
    /// smoothing) before the first turn of the single-phase loop.
    bool pre_smooth;
    /// DEBUG, 3D only. See the spec: which COLLAPSE test the ops guard uses. true (THE DEFAULT
    /// since 2026-09-23) is the pre-2026-09-22 one -- the maximum resolution measure over the two
    /// endpoints' offset faces before against the maximum over the survivor's after, refused on
    /// any strict rise. false is the per-face pairwise test, which is strictly stronger: it sees
    /// a face going from well under the bar to many times it, where the maximum is saturated by
    /// the worst face anywhere in the ring and cannot.
    bool debug_collapse_ring;
    /// 3D only (2D has no swap half to the guard). See the spec: how much of the
    /// resolution bar a flip of the offset surface must WIN for the guard to accept it. It gates
    /// the FLIP -- max sag after <= max sag before - this, and the cells under stop_energy, is
    /// the whole rule; one that misses the margin is refused rather than falling back on AMIPS.
    /// Without it the swap pass does not finish, on noise-sized flips that are all monotone.
    double flip_sag_margin;

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
        front_conv = json_params["front_conv"];
        front_conv_rel = json_params["front_conv_rel"];
        front_conv_criterion = json_params["front_conv_criterion"];
        stencil_order = json_params["stencil_order"];

        sorted_marching = json_params["sorted_marching"];
        sphere_trace_initialization = json_params["sphere_trace_initialization"];
        sphere_trace_target_rel_tol = json_params["sphere_trace_target_rel_tol"];
        experimental_consistent_construction_split =
            json_params["EXPERIMENTAL_consistent_construction_split"];
        experimental_aggresive_refine = json_params["EXPERIMENTAL_aggresive_refine"];
        output_path = json_params["output"];
        save_vtu = json_params["save_vtu"];
        phi_grid_resolution = json_params["phi_grid_resolution"];

        num_threads = json_params["num_threads"];
        max_iterations = json_params["max_iterations"];
        offset_envelope = json_params["offset_envelope"];
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
        pre_smooth = json_params["pre_smooth"];
        debug_collapse_ring = json_params["DEBUG_collapse_ring"];
        flip_sag_margin = json_params["flip_sag_margin"];

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

        // The convergence epsilon, the same absolute-or-relative pair as the envelope and
        // against the same reference. It is a length in space, so the bounding box diagonal is
        // the reference, not target_distance: tying the accuracy to the offset distance made
        // every change of target_distance a silent change of accuracy as well.
        if (front_conv > 0) {
            front_conv_rel = front_conv / diag_l;
        } else {
            front_conv = front_conv_rel * diag_l;
        }

        // An epsilon coarser than the offset it measures decides nothing: every front vertex is
        // "placed" and every face "resolved" from the first turn, whatever the offset looks like.
        // Checked on the resolved ABSOLUTE values, so it catches the mistake whichever of the two
        // forms the config used to state it.
        if (front_conv > target_distance) {
            log_and_throw_error(
                "front_conv {} must be <= target_distance {}: the convergence epsilon cannot be "
                "coarser than the offset distance it measures, or every front face reads as "
                "resolved from the first turn",
                front_conv,
                target_distance);
        }

        // The operation leash, the same absolute-or-relative pair as the envelope and the
        // convergence epsilon, against the same reference.
        if (offset_envelope > 0) {
            offset_envelope_rel = offset_envelope / diag_l;
        } else {
            offset_envelope = offset_envelope_rel * diag_l;
        }

        // l_min is relative to the offset distance rather than the bounding box: it is the offset
        // that has to be resolved. See the declaration.
        if (min_edge_length_rel < 0) {
            // The envelope eps expressed as a multiple of target_distance, which is what this
            // wants. offset_envelope_rel is a fraction of the BBOX DIAGONAL since 2026-09-24, so
            // the conversion goes through the resolved absolute rather than being the identity it
            // used to be -- the derived floor is unchanged in model units either way.
            min_edge_length_rel =
                std::max(offset_envelope / std::max(target_distance, 1e-16), 1e-12);
        }
        if (min_edge_length < 0) {
            min_edge_length = min_edge_length_rel * target_distance;
        } else {
            min_edge_length_rel = min_edge_length / std::max(target_distance, 1e-16);
        }
    }
};
} // namespace wmtk::components::topological_offset
