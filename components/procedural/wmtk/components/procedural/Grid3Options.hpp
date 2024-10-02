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
    friend void to_json(nlohmann::json& nlohmann_json_j, const Grid3Options& nlohmann_json_t)
    {
        nlohmann_json_j["tiling"] = tiling_names[static_cast<size_t>(nlohmann_json_t.tiling_type)];
        if (nlohmann_json_t.coordinates.has_value()) {
            nlohmann_json_j["coordinates"] = *nlohmann_json_t.coordinates;
        }
        nlohmann_json_j["dimensions"] = nlohmann_json_t.dimensions;
        {
            const auto& b = nlohmann_json_t.cycles;
            std::array<bool, 3> bs{{b[0], b[1], b[2]}};
            nlohmann_json_j["cycles"] = bs;
        }
    }
    friend void from_json(const nlohmann::json& nlohmann_json_j, Grid3Options& nlohmann_json_t)
    {
        nlohmann_json_t.dimensions = nlohmann_json_j["dimensions"].get<std::array<int64_t, 3>>();
        {
            const std::string tiling = nlohmann_json_j["tiling"];
            bool found = false;
            for (size_t j = 0; j < tiling_names.size(); ++j) {
                if (tiling == tiling_names[j]) {
                    found = true;
                    nlohmann_json_t.tiling_type = static_cast<TilingType>(j);
                }
            }
            if (!found) {
                throw std::runtime_error(fmt::format(
                    "Tiling type was not found, got [{}], expected one of {{[{}]}}",
                    tiling,
                    fmt::join(tiling_names, "],[")));
            }
        }
        {
            auto& b = nlohmann_json_t.cycles;
            const auto& c = nlohmann_json_j["cycles"];
            for (int j = 0; j < 3; ++j) {
                b[j] = c[j];
            }
        }
        if (const auto& coords = nlohmann_json_j["coordinates"]; !coords.is_null()) {
            if (nlohmann_json_j["coordinates"]["spacing"][0] > 0)
                nlohmann_json_t.coordinates = coords.get<Coordinates>();
        }
    }
};
} // namespace wmtk::components::procedural
