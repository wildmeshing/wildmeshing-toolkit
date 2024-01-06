#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct RegularSpaceOptions
{
    std::string input; // input mesh
    std::string output; // output mesh
    std::vector<std::string> pass_through;
    std::vector<std::tuple<std::string, int64_t, int64_t>> tags;
};

void to_json(nlohmann::json& j, RegularSpaceOptions& o);

void from_json(const nlohmann::json& j, RegularSpaceOptions& o);

} // namespace wmtk::components::internal