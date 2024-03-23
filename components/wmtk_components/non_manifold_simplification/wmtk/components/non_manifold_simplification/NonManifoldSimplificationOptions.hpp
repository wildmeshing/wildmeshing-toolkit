#pragma once

#include <wmtk/components/base/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {

struct NonManifoldSimplificationOptions
{
    std::string input;
    std::string output;
    std::string position;
    int64_t iterations;
    double length_abs;
    double length_rel;
    double envelope_size;
    std::string non_manifold_vertex_label;
    std::string non_manifold_edge_label;
    int64_t non_manifold_tag_value;
    std::vector<std::string> pass_through;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    NonManifoldSimplificationOptions,
    input,
    output,
    position,
    iterations,
    length_abs,
    length_rel,
    envelope_size,
    non_manifold_vertex_label,
    non_manifold_edge_label,
    non_manifold_tag_value,
    pass_through);

} // namespace components
} // namespace wmtk