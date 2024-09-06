#pragma once
#include <array>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
namespace wmtk::components::procedural {

class DiskOptions
{
public:
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
    std::optional<std::string> get_coordinate_name() const { if(coordinates.has_value()) { return coordinates.value().name;} else { return {}; } }
    friend void to_json(nlohmann::json& nlohmann_json_j, const DiskOptions& nlohmann_json_t)
    {
        nlohmann_json_j["size"] = nlohmann_json_t.size;
        if (nlohmann_json_t.coordinates.has_value()) {
            nlohmann_json_j["coordinates"] = *nlohmann_json_t.coordinates;
        }
    }
    friend void from_json(const nlohmann::json& nlohmann_json_j, DiskOptions& nlohmann_json_t)
    {
        nlohmann_json_t.size = nlohmann_json_j["size"];
        if (const auto& coords = nlohmann_json_j["coordinates"]; !coords.is_null()) {
            nlohmann_json_t.coordinates = coords.get<Coordinates>();
        }
    }
};
} // namespace wmtk::components::procedural
