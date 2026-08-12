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
    double relative_ball_threshold;
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
        pull_mode = json_params["pull_mode"];
        smooth_quality_gate = json_params["smooth_quality_gate"];
        two_stage = json_params["two_stage"];
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
    }
};
} // namespace wmtk::components::topological_offset
