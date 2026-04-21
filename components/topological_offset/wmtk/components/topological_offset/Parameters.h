#pragma once
#include <wmtk/Types.hpp>


namespace wmtk::components::topological_offset {
struct Parameters
{
    // parameters set by user
    std::vector<std::array<int, 2>> offset_tags;
    std::vector<std::array<int, 2>> offset_tag_val;
    double target_distance;
    double relative_ball_threshold;
    double edge_search_term_len;
    bool sorted_marching;
    std::string output_path; // no extension
    bool save_vtu = false;
    bool debug_output = false;
};
} // namespace wmtk::components::topological_offset
