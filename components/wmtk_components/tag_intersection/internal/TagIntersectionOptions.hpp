#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::components::internal {

struct TagIntersectionOptions
{
    std::string type; // tag_intersection
    std::string input;
    std::string output;
    std::vector<std::tuple<std::string, wmtk::PrimitiveType, long>> input_tags;
    std::vector<std::tuple<std::string, wmtk::PrimitiveType, long>> output_tags;
};

void to_json(nlohmann::json& j, TagIntersectionOptions& o);

void from_json(const nlohmann::json& j, TagIntersectionOptions& o);

} // namespace wmtk::components::internal