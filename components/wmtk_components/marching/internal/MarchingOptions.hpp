#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct MarchingOptions
{
    std::string type; // marching
    std::string input; // mesh input dir
    std::string output; // mesh output dir
    std::tuple<std::string, int64_t, int64_t> input_tags; // must be on vertex
    std::tuple<std::string, int64_t> output_vertex_tag;
    std::vector<std::tuple<std::string, int64_t>> edge_filter_tags;
};

// TODO move to cpp
inline void to_json(nlohmann::json& j, MarchingOptions& o)
{
    j = {
        {"type", o.type},
        {"input", o.input},
        {"output", o.output},
        {"input_tags", o.input_tags},
        {"output_tags", o.output_vertex_tag},
        {"edge_filter_tags", o.edge_filter_tags}};
}

// TODO move to cpp
inline void from_json(const nlohmann::json& j, MarchingOptions& o)
{
    o.type = j.at("type");
    if (o.type != "marching") {
        throw std::runtime_error("Wrong type in MarchingOptions");
    }

    o.input = j.at("input");
    o.output = j.at("output");
    j.at("input_tags").get_to(o.input_tags);
    j.at("output_tags").get_to(o.output_vertex_tag);
    j.at("edge_filter_tags").get_to(o.edge_filter_tags);
}

} // namespace internal
} // namespace components
} // namespace wmtk