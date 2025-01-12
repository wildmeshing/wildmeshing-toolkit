#include "AttributeDescription.hpp"
#include <nlohmann/json.hpp>
#include <fmt/format.h>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/components/multimesh/NamedMultiMesh.hpp>

#include <wmtk/components/utils/json_serialize_enum.hpp>
namespace wmtk::attribute {
// TODO: this definitely will cause a conflict someday if someone else wants to serialize
// attributetype
WMTK_NLOHMANN_JSON_SERIALIZE_ENUM(
    AttributeType,
    {
        {AttributeType::Char, "char"},
        {AttributeType::Double, "double"},
        {AttributeType::Int64, "int"},
        {AttributeType::Rational, "rational"},
    })
} // namespace wmtk::attribute

namespace wmtk::components::multimesh::utils {
auto AttributeDescription::operator<=>(const AttributeDescription&) const -> std::strong_ordering =
                                                                                 default;
auto AttributeDescription::operator==(const AttributeDescription&) const -> bool = default;

namespace {
NLOHMANN_JSON_SERIALIZE_ENUM(
    PrimitiveType,
    {
        {PrimitiveType::Vertex, 0},
        {PrimitiveType::Edge, 1},
        {PrimitiveType::Triangle, 2},
        {PrimitiveType::Tetrahedron, 3},
    })
} // namespace
WMTK_NLOHMANN_JSON_FRIEND_TO_JSON_PROTOTYPE(AttributeDescription)
{
    WMTK_NLOHMANN_ASSIGN_TYPE_TO_JSON(path);
    if (nlohmann_json_t.dimension.has_value()) {
        nlohmann_json_j["dimension"] = nlohmann_json_t.dimension.value();
    }
    if (nlohmann_json_t.type.has_value()) {
        nlohmann_json_j["type"] = nlohmann_json_t.type.value();
    }
}
WMTK_NLOHMANN_JSON_FRIEND_FROM_JSON_PROTOTYPE(AttributeDescription)
{
    if (nlohmann_json_j.is_string()) {
        nlohmann_json_t.path = nlohmann_json_j.get<std::string>();
    } else {
        nlohmann_json_t.path = nlohmann_json_j["path"];
    }
    if (nlohmann_json_j.contains("dimension")) {
        nlohmann_json_t.dimension = nlohmann_json_j["dimension"];
    }
    if (nlohmann_json_j.contains("type")) {
        nlohmann_json_t.type = nlohmann_json_j["type"];
    }
}

std::optional<PrimitiveType> AttributeDescription::primitive_type() const
{
    if (this->dimension.has_value()) {
        int8_t d = this->dimension.value();
        assert(d < 4);

        return get_primitive_type_from_id(d);
    } else {
        return {};
    }
}
AttributeDescription::AttributeDescription(const std::string_view& p, const wmtk::attribute::MeshAttributeHandle&mah): path(p), dimension(get_primitive_type_id(mah.primitive_type())), type(mah.held_type()) {}
AttributeDescription::AttributeDescription(const wmtk::attribute::MeshAttributeHandle&mah):AttributeDescription(mah.name(), mah) {
}
//AttributeDescription::AttributeDescription(const MeshCollection& mc, const wmtk::attribute::MeshAttributeHandle&mah): AttributeDescription(mc.get_path(mah), mah) {}
AttributeDescription::AttributeDescription(const NamedMultiMesh& mc, const wmtk::attribute::MeshAttributeHandle&mah): AttributeDescription(mc.get_path(mah), mah) {}

AttributeDescription::operator std::string() const {
    if(dimension.has_value() && type.has_value()) {
    return fmt::format("AttributeDescription({},t={},s={})", path, attribute::attribute_type_name(type.value()),dimension.value());
    } else if(dimension.has_value()) {
    return fmt::format("AttributeDescription({},s={})", path,dimension.value());
    } else if(type.has_value()) {
    return fmt::format("AttributeDescription({},t={})",path, attribute::attribute_type_name(type.value()));
    } else {
    return fmt::format("AttributeDescription({})", path);
    }
}

} // namespace wmtk::components::multimesh::utils
