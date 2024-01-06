#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct MarchingOptions
{
    std::string input; // mesh input dir
    std::string output; // mesh output dir
    std::vector<std::string> pass_through;
    std::tuple<std::string, int64_t, int64_t> input_tags; // must be on vertex
    std::tuple<std::string, int64_t> output_vertex_tag;
    std::vector<std::tuple<std::string, int64_t>> edge_filter_tags;
};

void to_json(nlohmann::json& j, MarchingOptions& o);

void from_json(const nlohmann::json& j, MarchingOptions& o);

} // namespace wmtk::components::internal