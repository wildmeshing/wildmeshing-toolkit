#pragma once
#include <wmtk/Types.hpp>


namespace wmtk::components::manifold_extraction {
struct Parameters
{
    // parameters set by user
    std::set<std::string> in_tag;
    std::set<std::string> replace_tag;
    bool manifold_union;
    std::string output_path;
    bool debug_output;
    bool write_surface;
};
} // namespace wmtk::components::manifold_extraction
