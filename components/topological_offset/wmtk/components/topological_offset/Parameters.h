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
 * unchanged: `length`/`length_rel` feed the base's `l`/`lr`, `smoothing_iterations` feeds
 * `num_smoothing_passes`, `DEBUG_output` feeds `debug_output`.
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
    double convergence_target; // absolute; if < 0, computed from convergence_target_rel in init()
    double convergence_target_rel; // relative to target_distance, not the bbox diagonal
    // Max normal deviation, in DEGREES, that the offset boundary must reach before the
    // optimization may terminate early. Absolute by nature -- an angle has no natural relative
    // form the way a distance does -- so there is no _rel counterpart. Convergence requires both
    // this and convergence_target; <= 0 disables the criterion, leaving distance alone deciding.
    double convergence_normal_deviation;
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
    double relative_ball_threshold;
    // Termination length for the distance-field root finds in EdgeSplittingTet.cpp
    // (edge_split_binary_search, edge_split_log_root_find, edge_split_sphere_tracing).
    // 3D ONLY. The 2D path has no consumer: its sphere-tracing split was removed along with the
    // distance-field marching pass that selected it, because placing the offset boundary is the
    // optimization phase's job, not the insertion's. Delete this field and its spec entry when
    // 3D drops those modes too -- see the note in .claude/CLAUDE.md.
    double edge_search_term_len;
    bool sorted_marching;
    std::string output_path; // no extension
    bool optimize; // whether to run optimization on the offset
    bool save_vtu;

    int num_threads; // number of threads for parallel execution (smoothing, collapse). 0 = serial
    int optimization_iterations; // number of split/collapse/swap/smooth passes in optimize_offset()

    // max angle (degrees, 0-90) allowed between an offset-surface face's own normal and the
    // input-complex normal it is supposed to approximate, before collapse/swap reject a move
    // that would push it further out of alignment.
    double max_normal_deviation_deg;
    // sigma_min from the paper (Sec. 5.3, "controls when the offset curvature is considered
    // planar"): a stretch of offset whose normal deviation is below this is flat enough that
    // the sizing field may coarsen it. Only the 2D sizing field reads it.
    double min_normal_deviation_deg;
    // l_min from the paper, = 2 * delta * sin(sigma_max): the shortest edge the sizing field
    // may ask for, in absolute units. Tied to the offset distance rather than the bounding box
    // because that is the scale the offset actually has. Derived in init() when < 0. This is a
    // floor on refinement, so raising it makes the result COARSER (paper Fig. 18).
    double min_edge_length;

    // ---- offset-surface smoothing blend, see TopoOffsetTetMesh::smooth_after_offset_surface()
    // ---- each offset-surface vertex moves to a weighted blend of its previous position,
    // the quadrics-optimal target vertex, and the Laplacian of its offset-surface
    // neighbors; the remaining weight (1 - w - u) stays with the previous position.
    double smooth_quadrics_weight; // w: blend toward the quadrics-optimal target vertex
    double smooth_laplacian_weight; // u: blend toward the offset-surface Laplacian
    // SVD threshold used by Quadrics::solve() when solving for the quadrics-optimal target
    // vertex. Controls sensitivity to feature edges: lower means more sensitive.
    double quadrics_svd_threshold;

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
        convergence_target = json_params["convergence_target"];
        convergence_target_rel = json_params["convergence_target_rel"];
        convergence_normal_deviation = json_params["convergence_normal_deviation"];
        throw_on_nonconvergence = json_params["throw_on_nonconvergence"];
        envelope_size = json_params["envelope_size"];
        envelope_size_rel = json_params["envelope_size_rel"];
        region_envelope_from_input = json_params["region_envelope_from_input"];
        relative_ball_threshold = json_params["relative_ball_threshold"];
        if (relative_ball_threshold < 0.0 || relative_ball_threshold > 1.0) {
            log_and_throw_error(
                "Invalid relative_ball_threshold [{}], must be between 0 and 1.",
                relative_ball_threshold);
        }

        edge_search_term_len = json_params["edge_search_termination_len"];
        sorted_marching = json_params["sorted_marching"];
        output_path = json_params["output"];
        optimize = json_params["optimize"];
        save_vtu = json_params["save_vtu"];

        num_threads = json_params["num_threads"];
        optimization_iterations = json_params["optimization_iterations"];

        max_normal_deviation_deg = json_params["max_normal_deviation_deg"];
        min_normal_deviation_deg = json_params["min_normal_deviation_deg"];
        min_edge_length = json_params["min_edge_length"];

        smooth_quadrics_weight = json_params["smooth_quadrics_weight"];
        smooth_laplacian_weight = json_params["smooth_laplacian_weight"];
        quadrics_svd_threshold = json_params["quadrics_svd_threshold"];

        min_sizing_scalar = json_params["min_sizing_scalar"];
        max_sizing_scalar = json_params["max_sizing_scalar"];
        sizing_mrm_threshold = json_params["sizing_mrm_threshold"];
        sizing_gradation = json_params["sizing_gradation"];

        // ---- inherited from wmtk::OptimizerParameters ----
        debug_output = json_params["DEBUG_output"];
        lr = json_params["length_rel"];
        l = json_params["length"];
        stop_energy = json_params["stop_energy"];
        num_smoothing_passes = json_params["smoothing_iterations"];
        split_high_valence_threshold = json_params["split_high_valence_threshold"];
        skip_good_regions = json_params["skip_good_regions"];
        w_amips = json_params["w_amips"];
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

        // The convergence threshold is an error in the SAME quantity target_distance measures --
        // how far an offset vertex sits from where it should be -- so it is relative to
        // target_distance, not to the bounding box diagonal like every other relative length.
        if (convergence_target > 0) {
            convergence_target_rel = convergence_target / target_distance;
        } else {
            convergence_target = convergence_target_rel * target_distance;
        }

        // An ordinary relative length, unlike convergence_target: it bounds how far a region
        // boundary may drift in space, so the bounding box is the right reference.
        if (envelope_size > 0) {
            envelope_size_rel = envelope_size / diag_l;
        } else {
            envelope_size = envelope_size_rel * diag_l;
        }

        // l_min = 2 * delta * sin(sigma_max), from the paper's parameter list (Sec. 5.3). The
        // reasoning is geometric: sigma_max is how far the offset surface is allowed to turn
        // across one element, and an element subtending that angle on a circle of radius delta
        // -- which is the shape the offset takes around a convex feature -- has chord length
        // 2*delta*sin(sigma_max). Scaling to the offset distance rather than the bounding box
        // is the point: it is the offset that has to be resolved, and its scale is delta.
        if (min_edge_length < 0) {
            const double sigma = max_normal_deviation_deg * M_PI / 180.;
            min_edge_length = 2. * target_distance * std::sin(std::min(sigma, M_PI / 2.));
        }
    }
};
} // namespace wmtk::components::topological_offset
