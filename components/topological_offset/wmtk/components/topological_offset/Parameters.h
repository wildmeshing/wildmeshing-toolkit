#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/expression_parser/Expression.hpp>

using ExpressionPtr = wmtk::components::simwild::expression_parser::ExpressionPtr;

namespace wmtk::components::topological_offset {
struct Parameters
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
    bool debug_output;

    int num_threads; // number of threads for parallel execution (smoothing, collapse). 0 = serial
    int smoothing_iterations; // number of smoothing passes run during optimize_offset()

    // ---- collapse, similar to SimWild ----
    double length_rel; // target edge length (relative to bbox diagonal)
    double length; // target edge length (absolute). If < 0, computed from length_rel in init()
    double stop_energy; // target AMIPS quality (see TopoOffsetTetMesh::get_quality()); a
                        // collapse that would push an already-on-target region's quality back
                        // above this is rejected

    VectorXd box_min;
    VectorXd box_max;

    // derived in init()
    double collapsing_l2; // upper bound (squared) on edge length for edge collapse eligibility
    double splitting_l2; // lower bound (squared) on edge length for edge split eligibility

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
        debug_output = json_params["DEBUG_output"];

        num_threads = json_params["num_threads"];
        smoothing_iterations = json_params["smoothing_iterations"];

        length_rel = json_params["length_rel"];
        length = json_params["length"];
        stop_energy = json_params["stop_energy"];
    }

    void init(const VectorXd& min_, const VectorXd& max_)
    {
        box_min = min_;
        box_max = max_;

        double diag_l = (max_ - min_).norm();
        if (target_distance > 0) {
            target_distance_rel = target_distance / diag_l;
        } else {
            target_distance = target_distance_rel * diag_l;
        }

        if (length > 0) {
            length_rel = length / diag_l;
        } else {
            length = length_rel * diag_l;
        }
        // only collapse edges shorter than 4/5 of the target length, matching SimWild
        collapsing_l2 = length * length * (16. / 25.);
        // only split edges longer than 4/3 of the target length, matching SimWild
        splitting_l2 = length * length * (16. / 9.);
    }
};
} // namespace wmtk::components::topological_offset
