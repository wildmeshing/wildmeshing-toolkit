#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct RegularSpaceOptions
{
    std::string type; // regular_space
    std::string input; // input mesh
    std::string output; // output mesh
    std::vector<std::tuple<std::string, int64_t, int64_t>> tags;
};

// TODO move to cpp
inline void to_json(nlohmann::json& j, RegularSpaceOptions& o)
{
    j = {{"type", o.type}, {"input", o.input}, {"output", o.output}, {"tags", o.tags}};
}

// TODO move to cpp
inline void from_json(const nlohmann::json& j, RegularSpaceOptions& o)
{
    o.type = j.at("type");
    if (o.type != "regular_space") {
        throw std::runtime_error("Wrong type in RegularSpaceOptions");
    }

    o.input = j.at("input");
    o.output = j.at("output");
    j.at("tags").get_to(o.tags);
}

} // namespace internal
} // namespace components
} // namespace wmtk