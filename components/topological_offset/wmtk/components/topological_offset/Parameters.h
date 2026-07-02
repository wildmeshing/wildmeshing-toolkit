#pragma once
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/expression_parser/Expression.hpp>

using ExpressionPtr = wmtk::components::simwild::expression_parser::ExpressionPtr;

namespace wmtk::components::topological_offset {
struct Parameters
{
    // std::vector<std::set<std::string>> offset_tags;
    ExpressionPtr offset_selection;
    std::set<std::string> offset_output_tag;
    // ExpressionPtr offset_output_tags;
    std::set<std::string> protected_tags;
    // ExpressionPtr protected_tags;

    bool respect_all_topologies;
    bool overwrite;
    bool offset_in;
    bool offset_out;
    double target_distance;
    double relative_ball_threshold;
    double edge_search_term_len;
    bool sorted_marching;
    std::string output_path; // no extension
    bool save_vtu = false;
    bool debug_output = false;
};
} // namespace wmtk::components::topological_offset
