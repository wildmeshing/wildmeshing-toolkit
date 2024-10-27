
#include "OutputOptions.hpp"
#include <fmt/std.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components::output {

WMTK_NLOHMANN_JSON_FRIEND_TO_JSON_PROTOTYPE(OutputOptions)
{
    //
    nlohmann_json_j["file"] = nlohmann_json_t.file.string();

    nlohmann_json_j["type"] = nlohmann_json_t.type;

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
        nlohmann_json_t.file = nlohmann_json_j.get<std::string>();
    } else {
    nlohmann_json_t.file = nlohmann_json_j["file"].get<std::string>();
    }
    if (nlohmann_json_j.contains("type")) {
        nlohmann_json_t.type = nlohmann_json_j["type"];
    } else {
        nlohmann_json_t.type = nlohmann_json_t.file.extension().string();
        wmtk::logger().debug("Guessing extension type of [{}] is [{}]", nlohmann_json_t.file, nlohmann_json_t.type);
    }
    if (nlohmann_json_j.contains("position_attribute")) {
        nlohmann_json_t.position_attribute = nlohmann_json_j["position_attribute"];
    }
}
} // namespace nlohmann
