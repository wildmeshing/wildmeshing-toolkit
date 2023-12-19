#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <wmtk/PrimitiveType.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct TagIntersectionOptions
{
    std::string type; // tag_intersection
    std::string input;
    std::string output;
    std::vector<std::tuple<std::string, wmtk::PrimitiveType, long>> input_tags;
    std::vector<std::tuple<std::string, wmtk::PrimitiveType, long>> output_tags;
};

// TODO move to cpp
inline void to_json(nlohmann::json& j, TagIntersectionOptions& o)
{
    j = {
        {"type", o.type},
        {"input", o.input},
        {"output", o.output},
        {"input_tags", o.input_tags},
        {"output_tags", o.output_tags}};
}

// TODO move to cpp
inline void from_json(const nlohmann::json& j, TagIntersectionOptions& o)
{
    o.type = j.at("type");
    if (o.type != "tag_intersection") {
        throw std::runtime_error("Wrong type in TagIntersectionOptions");
    }

    o.input = j.at("input");
    o.output = j.at("output");
    j.at("input_tags").get_to(o.input_tags);
    j.at("output_tags").get_to(o.output_tags);
}

} // namespace internal
} // namespace components
} // namespace wmtk