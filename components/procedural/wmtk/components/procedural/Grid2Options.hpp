#pragma once

#include <fmt/ranges.h>
#include <spdlog/spdlog.h>
#include <array>
#include <bitset>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace wmtk::components::procedural {
class Grid2Options
{
public:
    enum class TilingType { BCC = 0, Diagonal = 1 };
    const static std::array<std::string, 2> tiling_names;
    TilingType tiling_type;
    std::array<int64_t, 2> dimensions;
    std::bitset<2> cycles;
    struct Coordinates
    {
        std::string name;
        std::array<double, 2> spacing;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Coordinates, name, spacing);
    };
    std::optional<Coordinates> coordinates;
    std::optional<std::string> get_coordinate_name() const { if(coordinates.has_value()) { return coordinates.value().name;} else { return {}; } }

    friend void to_json(nlohmann::json& nlohmann_json_j, const Grid2Options& nlohmann_json_t);
    friend void from_json(const nlohmann::json& nlohmann_json_j, Grid2Options& nlohmann_json_t);
};
} // namespace wmtk::components::procedural
