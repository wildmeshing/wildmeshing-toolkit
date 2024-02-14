#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct RegularSpaceOptions
{
    std::string input; // input mesh
    std::string output; // output mesh
    nlohmann::json attributes;
    std::vector<int64_t> values;
    std::vector<std::string> pass_through;
};

void to_json(nlohmann::json& j, RegularSpaceOptions& o);

void from_json(const nlohmann::json& j, RegularSpaceOptions& o);

} // namespace wmtk::components::internal