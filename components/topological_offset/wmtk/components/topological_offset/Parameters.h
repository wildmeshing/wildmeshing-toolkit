#pragma once
#include <wmtk/Types.hpp>


namespace wmtk::components::topological_offset {
struct Parameters
{
    // parameters set by user
    std::string tag_name;
    std::vector<int> sep_tag_vals;
    int offset_tag_val;
    double target_distance;
    double relative_ball_threshold;
    std::string output_path; // no extension
    bool save_vtu = false;
    bool debug_output = false;
};
} // namespace wmtk::components::topological_offset
