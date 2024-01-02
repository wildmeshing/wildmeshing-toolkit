#pragma once

#include "MarchingOptions.hpp"

namespace wmtk::components::internal {

void to_json(nlohmann::json& j, MarchingOptions& o)
{
    j = {
        {"type", o.type},
        {"input", o.input},
        {"output", o.output},
        {"input_tags", o.input_tags},
        {"output_tags", o.output_vertex_tag},
        {"edge_filter_tags", o.edge_filter_tags}};
}

void from_json(const nlohmann::json& j, MarchingOptions& o)
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

} // namespace wmtk::components::internal