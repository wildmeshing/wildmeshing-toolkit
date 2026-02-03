#pragma once
#include <wmtk/Types.hpp>


namespace wmtk::components::manifold_extraction {
struct Parameters
{
    // parameters set by user
    std::string tag_label;
    std::array<double, 2> val_include_range;
    bool manifold_union;
    std::string output_path;
    bool debug_output = false;
};
} // namespace wmtk::components::manifold_extraction
