#pragma once

#include "RegularSpaceOptions.hpp"

namespace wmtk::components::internal {

inline void to_json(nlohmann::json& j, RegularSpaceOptions& o)
{
    j = {{"type", o.type}, {"input", o.input}, {"output", o.output}, {"tags", o.tags}};
}

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

} // namespace wmtk::components::internal