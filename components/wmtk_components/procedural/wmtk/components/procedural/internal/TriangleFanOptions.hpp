#pragma once
#include <array>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace wmtk::components::internal {
class TriangleFanOptions
{
public:
    int64_t size;
    struct Coordinates
    {
        std::string name;
        std::array<double, 2> center;
        double radius;
        std::array<double, 2> degrees; // in degrees

        friend void to_json(nlohmann::json& nlohmann_json_j, const Coordinates& nlohmann_json_t)
        {
            NLOHMANN_JSON_EXPAND(
                NLOHMANN_JSON_PASTE(NLOHMANN_JSON_TO, name, radius, center, degrees));
        }
        friend void from_json(const nlohmann::json& nlohmann_json_j, Coordinates& nlohmann_json_t)
        {
            NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_FROM, name, radius, center));
            std::cout << nlohmann_json_j.dump(2) << std::endl;
            assert(nlohmann_json_j.contains("degrees"));
            if (const auto& deg = nlohmann_json_j["degrees"]; deg.is_number()) {
                nlohmann_json_t.degrees = std::array<double, 2>{{0.0, double(deg)}};
            } else {
                nlohmann_json_t.degrees = deg.get<std::array<double, 2>>();
            }
        }
    };
    std::optional<Coordinates> coordinates;
    friend void to_json(nlohmann::json& nlohmann_json_j, const TriangleFanOptions& nlohmann_json_t)
    {
        nlohmann_json_j["size"] = nlohmann_json_t.size;
        if (nlohmann_json_t.coordinates.has_value()) {
            nlohmann_json_j["coordinates"] = *nlohmann_json_t.coordinates;
        }
    }
    friend void from_json(
        const nlohmann::json& nlohmann_json_j,
        TriangleFanOptions& nlohmann_json_t)
    {
        nlohmann_json_t.size = nlohmann_json_j["size"];
        if (const auto& coords = nlohmann_json_j["coordinates"]; !coords.is_null()) {
            nlohmann_json_t.coordinates = coords.get<Coordinates>();
        }
    }
};
} // namespace wmtk::components::internal
