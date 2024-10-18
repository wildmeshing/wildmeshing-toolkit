#pragma once
#include <array>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
namespace wmtk::components::procedural {

class DiskOptions
{
public:
    constexpr static auto name() -> std::string_view { return "disk"; }
    int64_t size;
    struct Coordinates
    {
        std::string name;
        std::array<double, 2> center;
        double radius;
        double degree_offset; // in degrees
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Coordinates, name, center, radius, degree_offset);
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
    void to_json(nlohmann::json& nlohmann_json_j, const DiskOptions& nlohmann_json_t);
    void from_json(const nlohmann::json& nlohmann_json_j, DiskOptions& nlohmann_json_t);
};
} // namespace wmtk::components::procedural
