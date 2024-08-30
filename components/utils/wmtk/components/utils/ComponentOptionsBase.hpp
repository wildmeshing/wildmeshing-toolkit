#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::utils {

struct ComponentOptionsBase
{
    std::string input; // input mesh
    std::string output; // output mesh
    nlohmann::json attributes; // necessary attributes for the component
    std::vector<std::string> pass_through; // further attributes that should not be deleted
};

} // namespace wmtk::components::utils