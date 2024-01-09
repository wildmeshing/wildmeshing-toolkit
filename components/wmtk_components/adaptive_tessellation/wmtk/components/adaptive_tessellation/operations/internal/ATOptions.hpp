#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct ATOptions
{
public:
    bool planar;
    int64_t passes;
    std::string input;
    std::string output;
    double target_edge_length;
    bool intermediate_output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ATOptions,
    planar,
    passes,
    input,
    output,
    target_edge_length,
    intermediate_output);

} // namespace wmtk::components