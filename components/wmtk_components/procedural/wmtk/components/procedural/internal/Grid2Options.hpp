#pragma once

#include <spdlog/spdlog.h>
#include <array>
#include <bitset>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace wmtk::components::internal {
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
    friend void to_json(nlohmann::json& nlohmann_json_j, const Grid2Options& nlohmann_json_t)
    {
        nlohmann_json_j["tiling"] = tiling_names[static_cast<size_t>(nlohmann_json_t.tiling_type)];
        if (nlohmann_json_t.coordinates.has_value()) {
            nlohmann_json_j["coordinates"] = *nlohmann_json_t.coordinates;
        }
        nlohmann_json_j["dimensions"] = nlohmann_json_t.dimensions;
        {
            const auto& b = nlohmann_json_t.cycles;
            std::array<bool,2> bs{{b[0],b[1]}};
            nlohmann_json_j["cycles"] = bs;
        }
    }
    friend void from_json(const nlohmann::json& nlohmann_json_j, Grid2Options& nlohmann_json_t)
    {
        nlohmann_json_t.dimensions = nlohmann_json_j["dimensions"].get<std::array<int64_t, 2>>();

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
                    "Tiling type was not found, got [{}], expected one of {[{}]}",
                    tiling,
                    fmt::join(tiling_names, "],[")));
            }
        }
        {
            auto& b = nlohmann_json_t.cycles;
            const auto& c = nlohmann_json_j["cycles"];
            for (int j = 0; j < 2; ++j) {
                b[j] = c[j];
            }
        }
        if (const auto& coords = nlohmann_json_j["coordinates"]; !coords.is_null()) {
            nlohmann_json_t.coordinates = coords.get<Coordinates>();
        }
    }
};
} // namespace wmtk::components::internal
