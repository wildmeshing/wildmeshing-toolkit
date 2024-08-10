#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
struct WildmeshingOptionsAttributes
{
    std::string position;
    bool replace_double_coordinate;
    //nlohmann::json position;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(WildmeshingOptionsAttributes, position, replace_double_coordinate);

struct WildmeshingOptionsEnvelopeMesh
{
    std::string mesh;
    std::string position;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(WildmeshingOptionsEnvelopeMesh, mesh, position);


struct WildmeshingOptionsEnvelope
{
    WildmeshingOptionsEnvelopeMesh geometry;
    double thickness;
    nlohmann::json constrained_position;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    WildmeshingOptionsEnvelope,
    geometry,
    thickness,
    constrained_position);

struct WildmeshingOptions
{
public:
    std::string input;
    std::string output;
    WildmeshingOptionsAttributes attributes;
    std::vector<nlohmann::json> pass_through;
    std::vector<WildmeshingOptionsEnvelope> envelopes;

    int64_t passes;
    double target_edge_length;
    double target_max_amips;
    bool intermediate_output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    WildmeshingOptions,
    attributes,
    pass_through,
    passes,
    input,
    envelopes,
    target_edge_length,
    target_max_amips,
    intermediate_output,
    output);

} // namespace wmtk::components
