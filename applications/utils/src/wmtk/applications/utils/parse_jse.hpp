#pragma once
#include <nlohmann/json.hpp>
#include <filesystem>

namespace wmtk::applications::utils {

    nlohmann::json parse_jse(
            const nlohmann::json& spec,
            const std::filesystem::path& path);
}
