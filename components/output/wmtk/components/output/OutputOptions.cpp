
#include "OutputOptions.hpp"
#include <fmt/std.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/components/utils/json_utils.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components::output {

WMTK_NLOHMANN_JSON_FRIEND_TO_JSON_PROTOTYPE(OutputOptions)
{
    WMTK_NLOHMANN_ASSIGN_TYPE_TO_JSON(path, type)
    if (nlohmann_json_t.mesh_name_path.has_value()) {
        nlohmann_json_j["mesh_name_path"] = nlohmann_json_t.mesh_name_path.value();
    }
    //
    // nlohmann_json_j["file"] = nlohmann_json_t.file.string();

    // nlohmann_json_j["type"] = nlohmann_json_t.type;

    nlohmann_json_j["position_attribute"] = std::visit(
        [](const auto& attr) -> std::string {
            using T = std::decay_t<decltype(attr)>;
            if constexpr (std::is_same_v<std::string, T>) {
                return attr;
            } else {
                return attr.mesh().get_attribute_name(attr.handle());
            }
        },
        nlohmann_json_t.position_attribute);
}
WMTK_NLOHMANN_JSON_FRIEND_FROM_JSON_PROTOTYPE(OutputOptions)
{
    if (nlohmann_json_j.is_string()) {
        nlohmann_json_t.path = nlohmann_json_j.get<std::filesystem::path>();
    } else if(nlohmann_json_j.contains("path")) {
        nlohmann_json_t.path = nlohmann_json_j["path"].get<std::filesystem::path>();
    } else if(nlohmann_json_j.contains("file")) {
        wmtk::logger().warn("OutputOptions using file is deprecated, use file");
        nlohmann_json_t.path = nlohmann_json_j["file"].get<std::filesystem::path>();
    }
    if (nlohmann_json_j.contains("type")) {
        nlohmann_json_t.type = nlohmann_json_j["type"];
    } else {
        nlohmann_json_t.type = nlohmann_json_t.path.extension().string();
        wmtk::logger().debug(
            "Guessing extension type of [{}] is [{}]",
            nlohmann_json_t.path,
            nlohmann_json_t.type);
    }
    if (nlohmann_json_j.contains("position_attribute")) {
        nlohmann_json_t.position_attribute = nlohmann_json_j["position_attribute"];
    }

    if (nlohmann_json_j.contains("mesh_name_path")) {
        nlohmann_json_t.mesh_name_path = nlohmann_json_j["mesh_name_path"].get<std::filesystem::path>();
    }
}
} // namespace wmtk::components::output
