#pragma once

#include "OutputOptions.hpp"

#include <nlohmann/json_fwd.hpp>
namespace wmtk::components::output {
    std::map<std::string, OutputOptions> parse_output(const nlohmann::json& js);
}
