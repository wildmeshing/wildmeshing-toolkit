#pragma once

#include <nlohmann/json.hpp>

#include <wmtk/io/Cache.hpp>

namespace wmtk::components {
wmtk::io::Cache run_components(const nlohmann::json& json_input_file, bool strict);
}