#pragma once
#include <array>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace wmtk::components::procedural {
class TriangleFanOptions
{
public:
    constexpr static auto name() -> std::string_view { return "triangle_fan"; }
    int64_t size;
    struct Coordinates
    {
        std::string name;
        std::array<double, 2> center;
        double radius;
        std::array<double, 2> degrees; // in degrees

        friend void to_json(nlohmann::json& nlohmann_json_j, const Coordinates& nlohmann_json_t);
        friend void from_json(const nlohmann::json& nlohmann_json_j, Coordinates& nlohmann_json_t);
    };
    std::optional<Coordinates> coordinates;

    std::optional<std::string> get_coordinate_name() const
    {
        if (coordinates.has_value()) {
            return coordinates.value().name;
        } else {
            return {};
        }
    }
    friend void to_json(nlohmann::json& nlohmann_json_j, const TriangleFanOptions& nlohmann_json_t);
    friend void from_json(
        const nlohmann::json& nlohmann_json_j,
        TriangleFanOptions& nlohmann_json_t);
};
} // namespace wmtk::components::procedural
