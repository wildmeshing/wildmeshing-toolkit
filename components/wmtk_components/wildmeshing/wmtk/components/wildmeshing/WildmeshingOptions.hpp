#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
struct WildmeshingOptionsAttributes
{
    std::string position;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(WildmeshingOptionsAttributes, position);

struct WildmeshingOptions
{
public:
    std::string input;
    std::string output;
    WildmeshingOptionsAttributes attributes;
    std::vector<std::string> pass_through;

    int64_t passes;
    double target_edge_length;
    bool intermediate_output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    WildmeshingOptions,
    attributes,
    pass_through,
    passes,
    input,
    target_edge_length,
    intermediate_output,
    output);

} // namespace wmtk::components
