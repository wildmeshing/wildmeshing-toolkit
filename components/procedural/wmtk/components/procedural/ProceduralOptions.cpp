#include "ProceduralOptions.hpp"


namespace wmtk::components::procedural {

    void to_json(nlohmann::json& nlohmann_json_j, const ProceduralOptions& nlohmann_json_t)
    {
        std::visit(
            [&](const auto& s) { nlohmann_json_j["settings"] = s; },
            nlohmann_json_t.settings);
    }
    void from_json(const nlohmann::json& nlohmann_json_j, ProceduralOptions& nlohmann_json_t)
    {
        if (nlohmann_json_j.contains("triangle_fan")) {
            const auto& settings_js = nlohmann_json_j["triangle_fan"];
            nlohmann_json_t.settings = settings_js.get<TriangleFanOptions>();
        } else if (nlohmann_json_j.contains("grid")) {
            const auto& settings_js = nlohmann_json_j["grid"];
            nlohmann_json_t.settings = settings_js.get<GridOptions>();
        } else if (nlohmann_json_j.contains("disk")) {
            const auto& settings_js = nlohmann_json_j["disk"];
            nlohmann_json_t.settings = settings_js.get<DiskOptions>();

        } else {
            throw std::runtime_error(fmt::format("Unknown procedural mesh mesh_type"));
        }
    }
}
