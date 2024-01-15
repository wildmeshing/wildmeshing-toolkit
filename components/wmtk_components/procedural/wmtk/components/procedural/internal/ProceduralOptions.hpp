#pragma once

#include <wmtk/components/base/json_utils.hpp>

#include <nlohmann/json.hpp>
#include "DiskOptions.hpp"
#include "GridOptions.hpp"
#include "TriangleFanOptions.hpp"

namespace wmtk::components::internal {


struct ProceduralOptions
{
    std::string name;
    std::variant<GridOptions, TriangleFanOptions, DiskOptions> settings;

    friend void to_json(nlohmann::json& nlohmann_json_j, const ProceduralOptions& nlohmann_json_t)
    {
        NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_TO, name));
        std::visit(
            [&](const auto& s) { nlohmann_json_j["settings"] = s; },
            nlohmann_json_t.settings);
    }
    friend void from_json(const nlohmann::json& nlohmann_json_j, ProceduralOptions& nlohmann_json_t)
    {
        NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_FROM, name));
        std::string mesh_type = nlohmann_json_j["mesh_type"];

        const auto& settings_js = nlohmann_json_j["settings"];
        if (mesh_type == "grid") {
            nlohmann_json_t.settings = settings_js.get<GridOptions>();
        } else if (mesh_type == "triangle_fan") {
            nlohmann_json_t.settings = settings_js.get<TriangleFanOptions>();
        } else if (mesh_type == "disk") {
            nlohmann_json_t.settings = settings_js.get<DiskOptions>();
        } else {
            throw std::runtime_error(fmt::format("Unknown procedural mesh mesh_type [{}]", mesh_type));
        }
    }

};


} // namespace wmtk::components::internal
