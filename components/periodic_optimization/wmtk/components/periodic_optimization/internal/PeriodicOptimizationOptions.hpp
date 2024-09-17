#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct PeriodicOptimizationOptions
{
public:
    std::string periodic_mesh;
    std::string position_mesh;
    std::string output;

    double envelope_size;
    double target_max_amips;
    double target_edge_length;
    int64_t passes;
    bool intermediate_output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PeriodicOptimizationOptions,
    periodic_mesh,
    position_mesh,
    output,
    envelope_size,
    target_max_amips,
    target_edge_length,
    passes,
    intermediate_output);

} // namespace wmtk::components