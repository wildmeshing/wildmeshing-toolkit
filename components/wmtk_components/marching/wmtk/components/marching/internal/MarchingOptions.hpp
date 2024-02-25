#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct MarchingAttributes
{
    std::string vertex_label; // on vertex
    std::vector<std::string> filter_labels; // on edge
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MarchingAttributes, vertex_label, filter_labels);

struct MarchingOptions
{
    std::string input; // mesh input dir
    std::string output; // mesh output dir
    MarchingAttributes attributes;
    std::vector<int64_t> input_values;
    int64_t output_value;
    double weight;
    std::vector<int64_t> filter_values;
    std::vector<std::string> pass_through;
};

void to_json(nlohmann::json& j, MarchingOptions& o);

void from_json(const nlohmann::json& j, MarchingOptions& o);

} // namespace wmtk::components::internal