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
    // Half-width of the envelope that holds the held surfaces (3D: the input complex boundary and
    // the domain wall, plus every other region boundary in the final pass; see the spec doc).
    // Absolute; if < 0, computed from envelope_size_rel (relative to the bbox diagonal).
    double envelope_size;
    double envelope_size_rel;

    // ---- the smooth offset potential ----
    // Support radius of the potential, as a multiple of target_distance. Must be > 1: the offset
    // level set has to lie strictly inside the support, or the vertices on it get no gradient. A
    // band vertex that travels past the support is a hard error, not a silently frozen vertex.
    double offset_dhat_factor;
    std::string offset_field; ///< "smooth" (Phi level set) or "euclidean" (exact distance)
    // ---- the two convergence epsilons (was the single front_conv_rel) ----
    // Both are ABSOLUTE LENGTHS, resolved in init(); if < 0 each is computed from its _rel twin,
    // which is a fraction of the BOUNDING BOX DIAGONAL -- the same absolute/relative pair as
    // envelope_size / envelope_size_rel, and deliberately NOT a fraction of target_distance any
    // more, so changing the offset distance no longer silently changes the accuracy. init()
    // refuses either one above target_distance: an epsilon coarser than the offset it measures
    // cannot decide anything.
    //
    // THE ONE BAR, in 2D and in 3D. One quantity is measured everywhere -- over a simplex's
    // stencil (a face's in 3D, a front chord's in 2D), the RMS of the distance to the level set
    // along the field (OffsetPotential::relative_residual(), which for the euclidean field is the
    // relative error (Phi - c)/c) -- and compared against this. A vertex is placed when that same
    // measure at the vertex alone is within it, which is the order-0 stencil, so the vertex
    // measure and the face (chord) measure are one measure at two sample counts. Replaces
    // vertex_conv / sag_conv, which split the two apart 2026-09-23. What the loop exits on is
    // front_measure's choice; the vertex measure is a diagnostic since 2026-09-25.
    double front_conv;
    double front_conv_rel;

    /// The convergence epsilon as a FRACTION of target_distance, which is the form the
    /// dimensionless relative error OffsetPotential::relative_residual() is compared against. A
    /// mean of squared relative errors is below front_conv_frac()^2 exactly when the same mean
    /// taken in lengths is below front_conv^2 -- the two differ by target_distance^2 on both
    /// sides -- so which form the code uses is a matter of where the division sits, not of what
    /// is being asked.
    double front_conv_frac() const { return front_conv / std::max(target_distance, 1e-16); }
    /// The energy rule of collapses and swaps (see the spec); false is a debugging switch.
    bool offset_collapse_veto = true, offset_swap_veto = true;
    /// The front vertices' smoothing veto on the per-cell energy (see the spec); the engine's
    /// smooth_quality_veto field is the interior vertices' (key offset_smooth_veto).
    bool offset_front_smooth_veto = false;
    /// The weight w of AMIPS in the per-cell energy and in the front smoother's objective:
    /// tet_energy = w AMIPS^3 + the offset terms in 3D, tri_energy = w AMIPS + the offset terms in
    /// 2D -- the engine's own cell quality in each (see the spec). 1 is the 1:1 energy of
    /// 2026-09-28.
    double offset_amips_weight = 1e-4;
    /// The collapse energy rule compares only the cells whose energy the collapse changes (see the
    /// spec); false compares the whole rings of v1 and v2 before against the survivor's after.
    bool offset_collapse_changed_cells = true;
    /// Log-only: per pass, the front vertices whose ring measure crosses the bar (see the spec).
    bool debug_crossings = false;
    /// What a collapse's surviving vertex keeps as its sizing scalar. true (the default): the
    /// smaller of the two, which is the shared engine's rule -- refinement then never relaxes
    /// behind a travelling front. false: the survivor's own.
    bool sizing_collapse_min = true;
    /// 3D: AMIPS is measured against each cell's stamped rest shape (plastic AMIPS) everywhere in
    /// the loop -- the smoother, the operation guards and the vetoes; false: against the regular
    /// tet. The final pass is always against the regular tet. See the spec doc.
    bool use_rest_pose = true;
    /// The outer loop's budget in turns. The loop leaves on the front test; this is only the
    /// guard.
    int max_rounds = 40;
    // Sampling density of THE measure -- both the criterion's and the energy's, in 2D and 3D --
    // and of the residual diagnostics that share the lattice. Order k puts these points on a
    // face (3D):
    //   0 -> 3, the CORNERS alone, so the measure is exactly the three vertices' placement error;
    //   k >= 1 -> the vertices of the triangle subdivided k-1 times by 4-way midpoint refinement
    //   plus the centroid of each of its 4^(k-1) sub-triangles, i.e. 4, 10, 31, 109, ...
    // and on a front chord (2D), the same rule one dimension down:
    //   0 -> 2, the ENDS alone;
    //   k >= 1 -> the vertices of the chord cut into 2^(k-1) pieces plus the midpoint of each
    //   piece, i.e. 3, 5, 9, 17, ...
    // The corners are IN the stencil, unlike the strictly interior lattice this replaces, because
    // the quantity measured is a distance to the level set rather than an interpolation error and
    // so is not identically zero there. Raising it costs a Phi value and gradient per sample, in
    // the operations' energy rules (TopoOffsetTetMesh::tet_energy(),
    // TopoOffsetTriMesh::tri_energy()) as well as in energy_criterion() and every smoothing solve.
    // See TopoOffsetTetMesh::for_each_face_sample, TopoOffsetTriMesh::for_each_edge_sample.
    int stencil_order;
    /// Which measure the loop exits on and refines by, in 3D and in 2D; see the
    /// spec. "vertex_ring" (the default): at each front vertex the RING MEASURE, the root mean
    /// square of the face measures (face_offset_term()) of its incident offset faces -- in 2D
    /// of the chord measures (edge_offset_term()) of its front chords -- every simplex weighted
    /// equally as in the per-cell energy. The loop exits when every ring measure is within the
    /// bar and nothing is unmeasurable, and the halving takes each vertex over the bar alone.
    /// "face": every offset face (2D: chord) within the bar and the halving at the corners of
    /// every face over it -- the rule of 2026-09-25, kept for comparison.
    std::string front_measure;
    bool sorted_marching;
    /// How the marching places the offset (see the spec). A target_distance below the maximum
    /// marchable distance is traced to; otherwise "max_marchable_fallback" traces to half the
    /// maximum marchable distance and "midpoint_fallback" splits every marched edge at its
    /// midpoint. See marching_tris() / marching_tets().
    std::string construction_mode = "max_marchable_fallback";
    double sphere_trace_target_rel_tol; ///< |d - D| <= tol x D ends the trace (D: see above)
    /// EXPERIMENTAL, default false pending more runs. Separates the two length gates so a split can
    /// never hand the collapse pass its own halves. Both passes measure r = L / (l x mean of the
    /// endpoints' sizing scalars); the split fires at r > 4/3 and the collapse at r < 4/5, so an
    /// edge with 4/3 < r < 8/5 splits into halves with r/2 < 4/5, which the collapse pass merges
    /// straight back. With this on, splitting_l2 becomes (8/5 l)^2 -- twice the collapse gate --
    /// and collapsing_l2 is untouched, so every half a split produces is at or above the collapse
    /// gate. A split is still never refused on quality; this is a length rule alone. Measured on
    /// the deliverable cube at target_distance_rel 1e-2 / front_conv_rel 1e-4, alignment off: 81%
    /// of the split candidates at the end of turn 7 (29068 of 35865) were inside that band, and
    /// turns 6-8 each ran ~36k splits and ~29k collapses while refinement touched 0 vertices.
    /// With it on: last-turn operations 66826 -> 19263, 8 -> 7 turns, 223 -> 126 s, final max
    /// AMIPS 8.69 -> 10.64, front faces 25816 -> 21042 under the same bar. Off by default until
    /// more runs confirm it; false keeps the TetWild gates.
    bool experimental_nonoverlapping_gates = false;
    /// Default true. Only when the target is not below the maximum marchable distance (marched
    /// as construction_mode says): the loop first converges under stencil_order without
    /// refinement, then under stencil_order with refinement. See the spec doc.
    bool init_optimize = true;
    /// EXPERIMENTAL (2026-10-02). The stencil_order of the init_optimize loop alone; -1 (the
    /// default) uses stencil_order there too. See the spec doc.
    int init_optimize_stencil_order = -1;
    /// Smoothing passes before the march that push the outer ends of the marched edges out to
    /// 2 x target_distance; they stop early once every outer end is beyond target_distance +
    /// front_conv. 0 (the default) = off. See the spec doc and
    /// TopoOffsetTetMesh::repulsion_smoothing() / TopoOffsetTriMesh::repulsion_smoothing().
    int repulsion_smoothing_passes = 0;
    /// 2D and 3D. After those passes, at most this many rounds of the loop's operations (split,
    /// collapse, swap, each with its smoothing) before the march, with no refinement and no split
    /// of a marched edge; same stop test. 0 (the default) = off. See the spec doc.
    int repulsion_rounds = 0;
    std::string output_path; // no extension
    bool save_vtu;

    // Samples per side of the grid the smooth offset potential is written on, beside the result,
    // for the viewer. 0 disables it. The whole domain in 2D, one plane through the box in 3D.
    int phi_grid_resolution;

    int num_threads; // number of threads for parallel execution (smoothing, collapse). 0 = serial
    /// Cap of the shared TriWild/TetWild loop, which now runs in exactly one place: the
    /// frozen-front finishing pass.
    int max_iterations;
    // The offset envelope half-width: the leash the front is kept inside during the frozen-front
    // final pass, built once before it (3D: build_offset_envelope(); 2D:
    // rebuild_offset_envelope()). Absolute-or-relative exactly as envelope_size / envelope_size_rel
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
    /// smoothing) before the first turn of the loop.
    bool pre_smooth;

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
        offset_field = json_params["offset_field"];
        front_conv = json_params["front_conv"];
        front_conv_rel = json_params["front_conv_rel"];
        stencil_order = json_params["stencil_order"];
        front_measure = json_params["front_measure"];

        sorted_marching = json_params["sorted_marching"];
        construction_mode = json_params["construction_mode"];
        sphere_trace_target_rel_tol = json_params["sphere_trace_target_rel_tol"];
        experimental_nonoverlapping_gates = json_params["EXPERIMENTAL_nonoverlapping_gates"];
        init_optimize = json_params["init_optimize"];
        init_optimize_stencil_order = json_params["EXPERIMENTAL_init_optimize_stencil_order"];
        repulsion_smoothing_passes = json_params["repulsion_smoothing_passes"];
        repulsion_rounds = json_params["repulsion_rounds"];
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
        offset_collapse_veto = json_params["offset_collapse_veto"];
        offset_swap_veto = json_params["offset_swap_veto"];
        offset_front_smooth_veto = json_params["offset_front_smooth_veto"];
        offset_amips_weight = json_params["offset_amips_weight"];
        offset_collapse_changed_cells = json_params["offset_collapse_changed_cells"];
        debug_crossings = json_params["DEBUG_crossings"];
        sizing_collapse_min = json_params["sizing_collapse_min"];
        use_rest_pose = json_params["use_rest_pose"];
        max_rounds = json_params["max_rounds"];
        w_amips = json_params["w_amips"];
        smoothing_mode = json_params["smoothing_mode"];
        project_line_search_steps = json_params["project_line_search_steps"];
        project_line_search_nested_steps = json_params["project_line_search_nested_steps"];
        smooth_quality_veto =
            json_params["offset_smooth_veto"]; // the engine's field, this component's key
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

        // The split gate moved out to twice the collapse gate, so no half a split produces is a
        // collapse candidate: splitting_l2 = (8/5 l)^2 against collapsing_l2 = (4/5 l)^2, which
        // is left where it is. The overlapping TetWild gates (4/3 and 4/5), still the default, let
        // the passes trade the same edges every turn -- see the declaration for the measurement.
        if (experimental_nonoverlapping_gates) {
            splitting_l2 = l * l * (64 / 25.);
        }

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
