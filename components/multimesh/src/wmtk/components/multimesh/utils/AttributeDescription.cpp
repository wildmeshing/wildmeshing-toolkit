#include "AttributeDescription.hpp"
#include <nlohmann/json.hpp>

namespace wmtk::attribute {
// TODO: this definitely will cause a conflict someday if someone else wants to serialize
// attributetype
NLOHMANN_JSON_SERIALIZE_ENUM(
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
    WMTK_NLOHMANN_ASSIGN_TYPE_TO_JSON(path, dimension, type);
}
WMTK_NLOHMANN_JSON_FRIEND_FROM_JSON_PROTOTYPE(AttributeDescription)
{
    WMTK_NLOHMANN_ASSIGN_TYPE_FROM_JSON(path, dimension, type);
}

PrimitiveType AttributeDescription::primitive_type() const
{
    assert(dimension < 4);

    return get_primitive_type_from_id(dimension);
}

} // namespace wmtk::components::multimesh::utils
