#pragma once
#include <wmtk/Types.hpp>


namespace wmtk::components::topological_offset {
struct Parameters
{
    // parameters set by user
    std::string tag_label;
    // std::vector<int> sep_tags;
    // int fill_tag;
    // bool manifold_mode;
    double wn_threshold;
    bool manifold_union;
    std::string output_path;
    bool debug_output = false;
};
} // namespace wmtk::components::topological_offset
