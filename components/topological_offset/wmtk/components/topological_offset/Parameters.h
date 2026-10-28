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
    // Half-width of the envelope that holds the held surfaces (3D: every region boundary, the
    // input complex's included, and the domain wall; see the spec doc).
    // Absolute; if <= 0, computed from envelope_size_rel (relative to the bbox diagonal) by
    // init(). The spec's defaults, so a Parameters built without init() (the unit tests) holds
    // defined values rather than uninitialized ones; its envelope_size is then -1, which the
    // envelope build refuses, so such a test sets envelope_size itself.
    double envelope_size = -1.0;
    double envelope_size_rel = 1e-3;

    // ---- the smooth offset potential ----
    // Support radius of the potential, as a multiple of target_distance. Must be > 1: the offset
    // level set has to lie strictly inside the support, or the vertices on it get no gradient. A
    // band vertex that travels past the support is a hard error, not a silently frozen vertex.
    double offset_dhat_factor;
    std::string offset_field; ///< "smooth" (Phi level set) or "euclidean" (exact distance)
    // ---- the convergence epsilon ----
    // front_conv is an ABSOLUTE LENGTH, resolved in init(); if <= 0 it is computed from
    // front_conv_rel, which since 2026-10-21 is a fraction of TARGET_DISTANCE -- the config's,
    // fixed for the run, every growth stage included (from 2026-09-23 to 2026-10-20 it was a
    // fraction of the bounding box diagonal). init() refuses a front_conv above target_distance:
    // an epsilon coarser than the offset it measures cannot decide anything.
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
    // 2D only: read by the 2D pipeline, which the app refuses until it is brought into lock-step
    // with 3D. No JSON key; each keeps the default its key had. The 2D energy rule of collapses
    // and swaps, the 2D front vertices' smoothing veto, and the 2D collapse rule over the changed
    // cells (TopoOffsetTriMesh). The 2D non-front veto reads the engine's smooth_quality_veto,
    // which keeps the engine's default, true, the default of the key offset_smooth_veto that set
    // it. 3D's rules are always on and read none of these (TopoOffsetTetMesh::tet_energy()).
    bool offset_collapse_veto = true, offset_swap_veto = true;
    bool offset_front_smooth_veto = false;
    bool offset_collapse_changed_cells = true;
    /// The weight w of AMIPS in the per-cell energy: in 3D the energy's AMIPS term,
    /// (w / stop_energy^3) vol AMIPS^3 (TopoOffsetTetMesh::cell_energy()); in 2D
    /// tri_energy = w AMIPS + the offset terms. See the spec. 100 since 2026-10-28 (1 before), as
    /// the spec's default.
    double offset_amips_weight = 100.;
    /// Log-only: per pass, the front vertices whose ring measure crosses the bar (see the spec).
    bool debug_crossings = false;
    /// EXPERIMENTAL_front_pocket_split (see the spec): before every smoothing pass of the loop,
    /// split every pinned cell until none is left (TopoOffsetTetMesh::front_pocket_split()).
    /// true by default since 2026-10-28, as the spec.
    bool front_pocket_split = true;
    /// EXPERIMENTAL_front_spanned_split (see the spec): front_pocket_split() splits spanned edges
    /// and faces too, not only pinned cells. true by default since 2026-10-28, as the spec.
    bool front_spanned_split = true;
    /// EXPERIMENTAL_exit_no_folds (see the spec): the loop does not exit while a real fold remains.
    /// true by default since 2026-10-28, as the spec.
    bool exit_no_folds = true;
    /// EXPERIMENTAL_label_cleanup (see the spec), 3D: after each operation group's operation
    /// passes, cells on the wrong side of the level set flip between band and non-band where the
    /// flip keeps both labels' topology (TopoOffsetTetMesh::label_cleanup()). "off",
    /// "one_sided", "one_sided_tips", "one_sided_at_target" or "energy_decrease" (the default since
    /// 2026-10-28, as the spec; it needs band_volume_rule "surface" and front_pocket_split).
    std::string label_cleanup = "energy_decrease";
    /// What a collapse's surviving vertex keeps as its sizing scalar. true (the default): the
    /// smaller of the two, which is the shared engine's rule -- refinement then never relaxes
    /// behind a travelling front. false: the survivor's own.
    bool sizing_collapse_min = true;
    /// 3D: the medium outside the band and the input complex is plastic during the loop -- its
    /// AMIPS term against a rest shape stamped before every operation group and smoothing block.
    /// false: equilateral AMIPS for every cell. See the spec doc.
    bool use_rest_pose = false;
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
    // so is not identically zero there. Raising it costs a Phi value per sample in
    // energy_criterion() and the frames, and in 2D also in the operations' energy rules
    // (TopoOffsetTriMesh::tri_energy()) and every front solve; the 3D energy has no face term.
    // See TopoOffsetTetMesh::for_each_face_sample, TopoOffsetTriMesh::for_each_edge_sample.
    int stencil_order = 5; // the spec's default
    /// EXPERIMENTAL_stencil_type (3D, 2026-10-28): which points stencil_order's face stencil
    /// holds. "interior": interior_face_stencil(), (i, j, l) / (k + 2), i, j, l >= 1. "fem" (the
    /// default since 2026-10-28, as the spec): fem_face_stencil(), the degree-k Lagrange nodes
    /// (i, j, l) / k, i, j, l >= 0
    /// (order 1 the corners, 2 + the edge midpoints, 3 + two per edge and the centroid), weighted
    /// by their lumped area weights (fem_face_stencil_weights()). See the spec doc. 2D does not
    /// read it.
    std::string stencil_type = "fem";
    /// Which measure the loop exits on and refines by, in 3D and in 2D; see the
    /// spec. "vertex_ring" (the default): at each front vertex the RING MEASURE, the root mean
    /// square of the face measures (face_offset_term()) of its incident offset faces -- in 2D
    /// of the chord measures (edge_offset_term()) of its front chords -- every simplex weighted
    /// equally. The loop exits when every ring measure is within the
    /// bar and nothing is unmeasurable, and the halving takes each vertex over the bar alone.
    /// "face": every offset face (2D: chord) within the bar and the halving at the corners of
    /// every face over it -- the rule of 2026-09-25, kept for comparison.
    std::string front_measure;
    bool sorted_marching;
    /// Construction only (optimize_offset false), 2D and 3D: the output .msh carries the node
    /// field corr_input_vid, each offset vertex's input-complex vertex (see the spec).
    bool save_offset_correspondence = false;
    /// How the marching places the offset (see the spec). A target_distance below the maximum
    /// marchable distance is traced to; otherwise "max_marchable_fallback" traces to half the
    /// maximum marchable distance and "midpoint_fallback" splits every marched edge at its
    /// midpoint. See marching_tris() / marching_tets().
    std::string construction_mode = "max_marchable_fallback";
    double sphere_trace_target_rel_tol; ///< |d - D| <= tol x D ends the trace (D: see above)
    /// Default false. Only when the target is not below the maximum marchable distance (marched
    /// as construction_mode says): the loop first converges under stencil_order without
    /// refinement, then under stencil_order with refinement. See the spec doc.
    bool init_optimize = false;
    // 2D only: read by the 2D pipeline, which the app refuses until it is brought into lock-step
    // with 3D. No JSON key; the default its key had. The stencil_order of the 2D init_optimize
    // loop alone; -1 uses stencil_order there too.
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
    /// EXPERIMENTAL (2026-10-12), 3D. The band term b(t) of a band cell in the energy
    /// (TopoOffsetTetMesh::band_cell_term()). "corner_bound" (the default): vol times the min
    /// over all input triangles P of the corner mean of (d_P - delta) / delta, an upper bound on
    /// the cell's part of int_B (d - delta) / delta dV that no split raises. "exact": J_t / delta,
    /// J_t = int_t (d - delta) dV in closed form (ExactBandIntegral); an upper bound only on its
    /// rule-(a) pieces. "front_flux": the operations as "exact", the front smoother's band term in
    /// the front form (flux of a field G with div G = d - delta through the band's boundary faces
    /// at the vertex, plus the bisector jumps D_t of its flagged cells). "surface" (2026-10-27):
    /// no band term; the offset term is a surface integral over the front, L /
    /// front_conv^2 (L the bounding box diagonal) sum over the offset faces of (A_f / |S|) sum over the interior stencil of
    /// (d - target_distance)^2 (TopoOffsetTetMesh::surface_face_term()). See the spec doc.
    /// "surface" is the default since 2026-10-28 ("corner_bound" before), as the spec.
    std::string band_volume_rule = "surface";
    /// EXPERIMENTAL_band_volume_rule "surface" only: the settle test's stall threshold. O* = the
    /// max over the front's stencil points of (d - target_distance)^2; a turn is settled when O*
    /// < front_conv^2 or O*_prev - O* <= settle_stall_eps (O* - front_conv^2) -- TetWild's stall
    /// test on the max offset error. See the spec doc.
    double settle_stall_eps = 0.1;
    /// EXPERIMENTAL_band_volume_rule "surface" only: which settle test. "stall" (the default):
    /// O* stall test above (settle_stall_eps). "bar": the volume rules' bar test on the surface
    /// integral -- settled when O_g = int over the front of (d - target_distance)^2 fell by less
    /// than front_conv^2 times the front's area, O_g of a front one front_conv off everywhere.
    std::string settle_test = "stall";
    /// EXPERIMENTAL (2026-10-09), 3D. Refinement and exit wait for the front to settle: a
    /// turn is settled when the offset energy (E without its AMIPS term) fell by less than the bar
    /// energy
    /// b = front_conv^2 A / (2 target_distance), A the front's area -- the
    /// offset term's excess for a front lying front_conv off the level set. On a settled turn
    /// every front face whose distance from the level set (the face's min of d, and its max of d,
    /// ExactBandIntegral::max_distance()) exceeds front_conv has its corners halved;
    /// exit = settled and no face over. The energy criterion's own refinement and exit are off.
    /// See the spec doc.
    bool settle_refinement = true;
    /// EXPERIMENTAL (2026-10-10), with EXPERIMENTAL_settle_refinement: on a settled turn a band
    /// cell with a front corner is also refined (its corners halved) when md - d(centroid) >
    /// front_conv, md the corner mean of the bounding triangle's distance (the corner bound's own
    /// min). Where d is convex on the cell, Vol (md - d(c)) / delta bounds the corner bound's gap
    /// (bound - true integral) from above (Jensen). Exit also needs no such cell. See the spec doc.
    bool gap_refinement = false;
    /// EXPERIMENTAL (2026-10-15), 3D: every smoothing solve of the loop may move its vertex at
    /// most damping_factor times the shortest edge at the vertex (to any vertex of its one-ring),
    /// measured from where the solve started (TopoOffsetTetMesh::damping_step(),
    /// SmoothVertexOptions::max_step). Never in a TetWild pass or the repulsion passes
    /// (TopoOffsetTetMesh::damping_active()). See the spec doc.
    bool damping = false;
    /// EXPERIMENTAL (2026-10-15), 3D, with damping: the bound as a factor of the shortest edge at
    /// the vertex. 0.5 (the default) = half, the bound damping had before the key existed.
    double damping_factor = 0.5;
    /// 3D, with optimize_offset (2026-10-21): before the loop, TetWild's own pass on the mesh as
    /// constructed, from the sizing field at 1 everywhere, the offset surface held in an envelope
    /// of half the distance the march traced it to (TopoOffsetTetMesh::optimize_construction_pass()).
    /// Ignored when optimize_offset is false. See the spec doc.
    bool optimize_construction = true;
    /// 3D (2026-10-21; "linear" was EXPERIMENTAL_incremental_growth): how the loop grows the
    /// offset when the march traces the front below target_distance (construction_mode
    /// "max_marchable_fallback", the target not below the maximum marchable distance), from the
    /// traced distance d_0. "none": the ordinary loop at target_distance. "linear": loops at d_0
    /// and at d_0 + k (target_distance - d_0) / linear_growth_steps, each to convergence, the last
    /// the ordinary loop (TopoOffsetTetMesh::linear_growth()). "smart": a loop at d_0, then
    /// attempts toward target_distance, reverted and halved when they fail
    /// (TopoOffsetTetMesh::smart_growth()). "individual_op_reject": loops at d_0 and at
    /// target_distance (linear with one step), every collapse, offset-surface flip and smoothing
    /// move that raises the folded offset faces refused (TopoOffsetTetMesh::fold_reject_active()).
    /// See the spec doc.
    std::string growth_mode = "none";
    /// growth_mode "linear": N, the steps from d_0 to target_distance, so N + 1 loops. At least 1
    /// (the spec's min).
    int linear_growth_steps = 4;
    /// growth_mode "smart": the turns an attempt gets to converge. At least 1 (the spec's min).
    int smart_growth_attempts = 2;
    /// growth_mode "smart": the failed attempts allowed from one converged distance before the run
    /// gives up (throws). At least 1 (the spec's min).
    int smart_growth_max_fails = 10;
    /// 3D: a front face is a foldover when the angle between its normal, out of the band, and
    /// the field's normal (grad d at its centroid) exceeds this many degrees
    /// (TopoOffsetTetMesh::front_face_field_angle()): the debug frames' per-face
    /// field_angle_foldover and the end-of-run [foldover] count. See the spec doc.
    double foldover_angle = 30.;
    /// DEBUG (2026-10-20), 3D: debug frames for a remote machine short on space -- overrides
    /// DEBUG_output (sets it, so every debug computation runs) and writes only the constructed
    /// mesh, each TetWild pass's start and every iteration of it, the end of the construction
    /// TetWild pass, the end of every turn of the loop, and the final mesh. See the spec doc.
    bool debug_remote_output = false;
    /// TESTING (2026-10-16), 3D: an input complex of more than one connected piece is accepted;
    /// the energy's band term then reads the distance to all pieces at once. See the spec doc.
    bool allow_mult_comp = false;
    /// EXPERIMENTAL (2026-10-09), read only under settle_refinement: the test that marks the front
    /// on a settled turn. "vertex": a front vertex's own |d - target_distance| exceeds
    /// front_conv; its sizing is halved. "vertex_edge" (the default): "vertex", and also both ends
    /// of a front edge whose ends are within front_conv and whose midpoint is not have their sizing
    /// halved (they do not hold the exit). "vertex_split": "vertex", and also a front face whose
    /// corners are within front_conv but an edge midpoint or its centroid is not has its worst edge
    /// split before the next turn's first smoothing pass (needs EXPERIMENTAL_front_pocket_split).
    /// "max": a face's max of
    /// |d - target_distance| exceeds front_conv (exact up to rounding; an upper bound on rule-(a)
    /// pieces, see ExactBandIntegral::max_distance()). "rms": a face's share of the offset excess
    /// exceeds its bar energy, X_f = (1/2 delta) int_f (d - delta)^2 cos(theta) dA >
    /// front_conv^2 area(f) / (2 delta), i.e. the mean over f of (d - delta)^2 cos(theta) exceeds
    /// front_conv^2 (theta between the face's normal, out of the band, and grad d; signed), by the
    /// 3-edge-midpoint rule. See the spec doc.
    std::string settle_mark = "vertex_edge";
    std::string output_path; // no extension
    bool save_vtu;

    // Samples per side of the grid the smooth offset potential is written on, beside the result,
    // for the viewer. 0 disables it. The whole domain in 2D, one plane through the box in 3D.
    int phi_grid_resolution;

    int num_threads; // number of threads for parallel execution (smoothing, collapse). 0 = serial
    /// Cap of the shared TriWild/TetWild loop, which runs only in the TetWild passes: the final
    /// pass, and in 3D the construction pass (optimize_construction).
    int max_iterations;
    // NO KEY since 2026-10-20 (offset_envelope / offset_envelope_rel removed): the offset
    // surface's envelope in a TetWild pass has envelope_size's eps, like every other held surface
    // (optimize_construction's pass alone widens it). init() sets these two to
    // envelope_size / envelope_size_rel; only the 2D code still reads them.
    double offset_envelope = -1.;
    double offset_envelope_rel = -1.;

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
    /// See the spec: true runs one smoothing block (the fixed interleaved count) before the first
    /// turn of the loop.
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
        stencil_type = json_params["EXPERIMENTAL_stencil_type"];
        if (stencil_type != "interior" && stencil_type != "fem") {
            log_and_throw_error(
                "EXPERIMENTAL_stencil_type is \"interior\" or \"fem\", not \"{}\"",
                stencil_type);
        }
        front_measure = json_params["front_measure"];

        sorted_marching = json_params["sorted_marching"];
        save_offset_correspondence = json_params["save_offset_correspondence"];
        construction_mode = json_params["construction_mode"];
        sphere_trace_target_rel_tol = json_params["sphere_trace_target_rel_tol"];
        init_optimize = json_params["init_optimize"];
        repulsion_smoothing_passes = json_params["repulsion_smoothing_passes"];
        repulsion_rounds = json_params["repulsion_rounds"];
        band_volume_rule = json_params["EXPERIMENTAL_band_volume_rule"];
        if (band_volume_rule != "corner_bound" && band_volume_rule != "exact" &&
            band_volume_rule != "front_flux" && band_volume_rule != "surface") {
            log_and_throw_error(
                "EXPERIMENTAL_band_volume_rule is \"corner_bound\", \"exact\", \"front_flux\" or "
                "\"surface\", not \"{}\"",
                band_volume_rule);
        }
        settle_stall_eps = json_params["settle_stall_eps"];
        settle_test = json_params["EXPERIMENTAL_settle_test"];
        if (settle_test != "stall" && settle_test != "bar") {
            log_and_throw_error(
                "EXPERIMENTAL_settle_test is \"stall\" or \"bar\", not \"{}\"",
                settle_test);
        }
        if (band_volume_rule != "surface" && settle_test != "stall") {
            log_and_throw_error(
                "EXPERIMENTAL_settle_test \"{}\" is read only under "
                "EXPERIMENTAL_band_volume_rule \"surface\" (this config has \"{}\"): leave it "
                "out",
                settle_test,
                band_volume_rule);
        }
        if (settle_test == "bar" && settle_stall_eps != 0.1) {
            log_and_throw_error(
                "settle_stall_eps ({}) is read only by EXPERIMENTAL_settle_test \"stall\": leave "
                "it out under \"bar\"",
                settle_stall_eps);
        }
        if (band_volume_rule != "surface" && settle_stall_eps != 0.1) {
            log_and_throw_error(
                "settle_stall_eps ({}) is read only under EXPERIMENTAL_band_volume_rule "
                "\"surface\" (this config has \"{}\"): leave it out",
                settle_stall_eps,
                band_volume_rule);
        }
        settle_refinement = json_params["EXPERIMENTAL_settle_refinement"];
        gap_refinement = json_params["EXPERIMENTAL_gap_refinement"];
        if (gap_refinement && !settle_refinement) {
            log_and_throw_error("EXPERIMENTAL_gap_refinement needs EXPERIMENTAL_settle_refinement");
        }
        damping = json_params["EXPERIMENTAL_damping"];
        damping_factor = json_params["EXPERIMENTAL_damping_factor"];
        if (!(damping_factor > 0.)) {
            log_and_throw_error("EXPERIMENTAL_damping_factor must be > 0, not {}", damping_factor);
        }
        // 0.5 is the spec's default: a factor set to anything else means damping was intended.
        if (!damping && damping_factor != 0.5) {
            log_and_throw_error(
                "EXPERIMENTAL_damping_factor {} needs EXPERIMENTAL_damping true",
                damping_factor);
        }
        optimize_construction = json_params["optimize_construction"];
        growth_mode = json_params["growth_mode"];
        linear_growth_steps = json_params["linear_growth_steps"];
        smart_growth_attempts = json_params["smart_growth_attempts"];
        smart_growth_max_fails = json_params["smart_growth_max_fails"];
        if (growth_mode != "none" && growth_mode != "linear" && growth_mode != "smart" &&
            growth_mode != "individual_op_reject") {
            log_and_throw_error(
                "growth_mode is \"none\", \"linear\", \"smart\" or \"individual_op_reject\", "
                "not \"{}\"",
                growth_mode);
        }
        // The spec's defaults: a value set to anything else means that mode was intended.
        if (growth_mode != "linear" && linear_growth_steps != 4) {
            log_and_throw_error(
                "linear_growth_steps {} needs growth_mode \"linear\", not \"{}\"",
                linear_growth_steps,
                growth_mode);
        }
        if (growth_mode != "smart" &&
            (smart_growth_attempts != 2 || smart_growth_max_fails != 10)) {
            log_and_throw_error(
                "smart_growth_attempts {} / smart_growth_max_fails {} need growth_mode \"smart\", "
                "not \"{}\"",
                smart_growth_attempts,
                smart_growth_max_fails,
                growth_mode);
        }
        // Only max_marchable_fallback traces the front below the target, where growth starts;
        // midpoint_fallback never does.
        if (growth_mode != "none" && construction_mode != "max_marchable_fallback") {
            log_and_throw_error(
                "growth_mode \"{}\" needs construction_mode \"max_marchable_fallback\", not "
                "\"{}\": growth starts at the distance the march traced the front to, and only "
                "that mode traces below target_distance",
                growth_mode,
                construction_mode);
        }
        if (growth_mode != "none" && init_optimize) {
            log_and_throw_error(
                "growth_mode \"{}\" and init_optimize both decide the loops that follow a march "
                "below target_distance: set only one of them",
                growth_mode);
        }
        foldover_angle = json_params["foldover_angle"];
        allow_mult_comp = json_params["TESTING_allow_mult_comp"];
        settle_mark = json_params["EXPERIMENTAL_settle_mark"];
        if (settle_mark != "max" && settle_mark != "rms" && settle_mark != "vertex" &&
            settle_mark != "vertex_edge" && settle_mark != "vertex_split") {
            log_and_throw_error(
                "EXPERIMENTAL_settle_mark is \"max\", \"rms\", \"vertex\", \"vertex_edge\" or "
                "\"vertex_split\", not \"{}\"",
                settle_mark);
        }
        output_path = json_params["output"];
        save_vtu = json_params["save_vtu"];
        phi_grid_resolution = json_params["phi_grid_resolution"];

        num_threads = json_params["num_threads"];
        max_iterations = json_params["max_iterations"];

        min_edge_length = json_params["min_edge_length"];
        min_edge_length_rel = json_params["min_edge_length_rel"];

        min_sizing_scalar = json_params["min_sizing_scalar"];
        max_sizing_scalar = json_params["max_sizing_scalar"];
        sizing_gradation = json_params["sizing_gradation"];
        sizing_gradation_mode = json_params["sizing_gradation_mode"];
        pre_smooth = json_params["pre_smooth"];

        // ---- inherited from wmtk::OptimizerParameters ----
        debug_output = json_params["DEBUG_output"];
        debug_remote_output = json_params["DEBUG_remote_output"];
        // DEBUG_remote_output overrides DEBUG_output: everything debug output computes is on, and
        // the frames follow the remote schedule (TopoOffsetTetMesh::remote_frames()).
        if (debug_remote_output) debug_output = true;
        lr = json_params["length_rel"];
        l = json_params["length"];
        stop_energy = json_params["stop_energy"];
        num_smoothing_passes = json_params["num_smoothing_passes"];
        interleaved_smoothing = json_params["interleaved_smoothing"];
        interleaved_smoothing_passes = json_params["interleaved_smoothing_passes"];
        colored_smoothing = json_params["colored_smoothing"];
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
        offset_amips_weight = json_params["offset_amips_weight"];
        debug_crossings = json_params["DEBUG_crossings"];
        front_pocket_split = json_params["EXPERIMENTAL_front_pocket_split"];
        front_spanned_split = json_params["EXPERIMENTAL_front_spanned_split"];
        exit_no_folds = json_params["EXPERIMENTAL_exit_no_folds"];
        label_cleanup = json_params["EXPERIMENTAL_label_cleanup"];
        if (label_cleanup != "off" && label_cleanup != "one_sided" &&
            label_cleanup != "one_sided_tips" && label_cleanup != "one_sided_at_target" &&
            label_cleanup != "energy_decrease") {
            log_and_throw_error(
                "EXPERIMENTAL_label_cleanup is \"off\", \"one_sided\", \"one_sided_tips\", "
                "\"one_sided_at_target\" or \"energy_decrease\", not \"{}\"",
                label_cleanup);
        }
        // "energy_decrease" reads the face term of EXPERIMENTAL_band_volume_rule "surface".
        if (label_cleanup == "energy_decrease" && band_volume_rule != "surface") {
            log_and_throw_error(
                "EXPERIMENTAL_label_cleanup \"energy_decrease\" needs "
                "EXPERIMENTAL_band_volume_rule \"surface\" (this config has \"{}\")",
                band_volume_rule);
        }
        // The clean-up runs in loop_group()'s EXPERIMENTAL_front_pocket_split path, between the
        // operation passes and the condition-1 splits before the first smoothing pass; the other
        // path has no such point.
        if (label_cleanup != "off" && !front_pocket_split) {
            log_and_throw_error(
                "EXPERIMENTAL_label_cleanup \"{}\" needs EXPERIMENTAL_front_pocket_split true: it "
                "runs right before the condition-1 splits that precede the first smoothing pass "
                "of each operation group",
                label_cleanup);
        }
        // "vertex_split" splits its edges in loop_group()'s EXPERIMENTAL_front_pocket_split path,
        // before the first smoothing pass of the next turn; the other path has no such point.
        if (settle_mark == "vertex_split" && !front_pocket_split) {
            log_and_throw_error(
                "EXPERIMENTAL_settle_mark \"vertex_split\" needs EXPERIMENTAL_front_pocket_split "
                "true: its splits are made right before the first smoothing pass of the next turn");
        }
        sizing_collapse_min = json_params["sizing_collapse_min"];
        use_rest_pose = json_params["use_rest_pose"];
        max_rounds = json_params["max_rounds"];
        w_amips = json_params["w_amips"];
        smoothing_mode = json_params["smoothing_mode"];
        project_line_search_steps = json_params["project_line_search_steps"];
        project_line_search_nested_steps = json_params["project_line_search_nested_steps"];
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

        // The convergence epsilon, absolute or relative; relative to target_distance (resolved
        // above), the offset it measures. A growth stage changes target_distance later, never
        // this.
        if (front_conv > 0) {
            front_conv_rel = front_conv / target_distance;
        } else {
            front_conv = front_conv_rel * target_distance;
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

        // The offset surface's envelope is envelope_size's (no key of its own since 2026-10-20);
        // the two fields only for the 2D code, which still reads them.
        offset_envelope = envelope_size;
        offset_envelope_rel = envelope_size_rel;

        // l_min is relative to the offset distance rather than the bounding box: it is the offset
        // that has to be resolved. See the declaration.
        if (min_edge_length_rel < 0) {
            // The envelope eps expressed as a multiple of target_distance, which is what this
            // wants: envelope_size, the offset surface's envelope too since offset_envelope went
            // (2026-10-20), resolved to an absolute length above.
            min_edge_length_rel = std::max(envelope_size / std::max(target_distance, 1e-16), 1e-12);
        }
        if (min_edge_length < 0) {
            min_edge_length = min_edge_length_rel * target_distance;
        } else {
            min_edge_length_rel = min_edge_length / std::max(target_distance, 1e-16);
        }
    }
};
} // namespace wmtk::components::topological_offset
