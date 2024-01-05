#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct WildmeshingOptions
{
public:
    int64_t passes;
    std::string input;
    double target_edge_length;
    bool intermediate_output;
    std::string output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    WildmeshingOptions,
    passes,
    input,
    target_edge_length,
    intermediate_output,
    output);

} // namespace wmtk::components
