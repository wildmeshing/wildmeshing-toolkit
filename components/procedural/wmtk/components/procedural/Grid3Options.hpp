#pragma once
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>
#include <array>
#include <bitset>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace wmtk::components::procedural {
class Grid3Options
{
public:
    enum class TilingType { BCC = 0, Freudenthal = 1 };
    const static std::array<std::string, 2> tiling_names;
    TilingType tiling_type;
    std::array<int64_t, 3> dimensions;
    std::bitset<3> cycles;
    struct Coordinates
    {
        std::string name;
        std::array<double, 3> spacing;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Coordinates, name, spacing);
    };
    std::optional<Coordinates> coordinates;
    std::optional<std::string> get_coordinate_name() const { if(coordinates.has_value()) { return coordinates.value().name;} else { return {}; } }
    friend void to_json(nlohmann::json& nlohmann_json_j, const Grid3Options& nlohmann_json_t);
    friend void from_json(const nlohmann::json& nlohmann_json_j, Grid3Options& nlohmann_json_t);
};
} // namespace wmtk::components::procedural
