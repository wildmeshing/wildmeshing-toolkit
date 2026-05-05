#pragma once
#include <wmtk/Types.hpp>


namespace wmtk::components::manifold_extraction {
struct Parameters
{
    // parameters set by user
    std::set<int64_t> in_tag;
    std::set<int64_t> replace_tag;
    bool manifold_union;
    std::string output_path;
    bool debug_output;
    bool write_surface;
};
} // namespace wmtk::components::manifold_extraction
