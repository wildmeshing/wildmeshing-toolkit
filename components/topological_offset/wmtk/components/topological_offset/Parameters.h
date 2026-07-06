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
    bool overwrite;
    bool offset_in;
    bool offset_out;
    double target_distance;
    double target_distance_rel;
    double relative_ball_threshold;
    double edge_search_term_len;
    bool sorted_marching;
    std::string output_path; // no extension
    bool save_vtu;
    bool debug_output;

    Parameters() = default;

    Parameters(const nlohmann::json& json_params)
    {
        for (const std::string& tag : json_params["offset_output_tags"]) {
            offset_output_tag.insert(tag);
        }
        for (const std::string& tag : json_params["protected_tags"]) {
            protected_tags.insert(tag);
        }
        respect_all_topologies = json_params["respect_all_topologies"];
        overwrite = json_params["overwrite_tags"];
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
        save_vtu = json_params["save_vtu"];
        debug_output = json_params["DEBUG_output"];
    }

    void init(const VectorXd& min_, const VectorXd& max_)
    {
        double diag_l = (max_ - min_).norm();
        if (target_distance > 0) {
            target_distance_rel = target_distance / diag_l;
        } else {
            target_distance = target_distance_rel * diag_l;
        }
    }
};
} // namespace wmtk::components::topological_offset
