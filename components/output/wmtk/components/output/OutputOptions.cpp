
#include "OutputOptions.hpp"
#include <fmt/std.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components::output {

//bool OutputOptions::operator==(const OutputOptions& o) const = default;
}

namespace nlohmann {
void adl_serializer<wmtk::components::output::OutputOptions>::to_json(json& j, const Type& v)
{
    //
    j["file"] = v.file.string();

    j["type"] = v.type;

    j["position_attribute"] = std::visit(
        [](const auto& attr) -> std::string {
            using T = std::decay_t<decltype(attr)>;
            if constexpr (std::is_same_v<std::string, T>) {
                return attr;
            } else {
                return attr.mesh().get_attribute_name(attr.handle());
            }
        },
        v.position_attribute);
}
void adl_serializer<wmtk::components::output::OutputOptions>::from_json(const json& j, Type& v)
{
    if (j.is_string()) {
        v.file = j.get<std::string>();
    } else {
    v.file = j["file"].get<std::string>();
    }
    if (j.contains("type")) {
        v.type = j["type"];
    } else {
        v.type = v.file.extension().string();
        wmtk::logger().debug("Guessing extension type of [{}] is [{}]", v.file, v.type);
    }
    if (j.contains("position_attribute")) {
        v.position_attribute = j["position_attribute"];
    }
}
} // namespace nlohmann
