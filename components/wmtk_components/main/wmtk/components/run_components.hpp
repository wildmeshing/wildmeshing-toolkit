#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
void run_components(const nlohmann::json& json_input_file, bool strict);
}