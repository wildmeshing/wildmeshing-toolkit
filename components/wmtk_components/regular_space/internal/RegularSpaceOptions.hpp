#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct RegularSpaceOptions
{
    std::string type; // regular_space
    std::string input; // input mesh
    std::string output; // output mesh
    std::vector<std::tuple<std::string, int64_t, int64_t>> tags;
};

inline void to_json(nlohmann::json& j, RegularSpaceOptions& o);

inline void from_json(const nlohmann::json& j, RegularSpaceOptions& o);

} // namespace wmtk::components::internal