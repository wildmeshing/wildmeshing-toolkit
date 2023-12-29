#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct WildmeshingOptions
{
public:
    bool planar;
    long passes;
    std::string input;
    double target_edge_length;
    bool intermediate_output;
    std::string filename;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    WildmeshingOptions,
    planar,
    passes,
    input,
    target_edge_length,
    intermediate_output,
    filename);

} // namespace wmtk::components
