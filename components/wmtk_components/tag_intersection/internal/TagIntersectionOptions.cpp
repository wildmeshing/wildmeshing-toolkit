#include "TagIntersectionOptions.hpp"

namespace wmtk::components::internal {

void to_json(nlohmann::json& j, TagIntersectionOptions& o)
{
    j = {
        {"type", o.type},
        {"input", o.input},
        {"output", o.output},
        {"input_tags", o.input_tags},
        {"output_tags", o.output_tags}};
}

void from_json(const nlohmann::json& j, TagIntersectionOptions& o)
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

} // namespace wmtk::components::internal