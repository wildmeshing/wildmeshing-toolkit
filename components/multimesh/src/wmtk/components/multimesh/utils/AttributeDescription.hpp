#pragma once
#include <compare>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/attribute/AttributeType.hpp>
#include <wmtk/components/utils/json_macros.hpp>

namespace wmtk::components::multimesh::utils {


// the minimal information to uniquely extract an attribute handle
struct AttributeDescription
{
    std::string path;
    uint8_t dimension; // internally the primitive type
    attribute::AttributeType type;

    PrimitiveType primitive_type() const;

    auto operator<=>(const AttributeDescription&) const -> std::strong_ordering;
    auto operator==(const AttributeDescription&) const -> bool;


    WMTK_NLOHMANN_JSON_FRIEND_DECLARATION(AttributeDescription)
};
} // namespace wmtk::components::multimesh::utils
