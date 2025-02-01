#pragma once
#include <cstdint>
#include <optional>
#include <wmtk/attribute/AttributeType.hpp>
namespace wmtk::components::multimesh::utils {
struct AttributeDescription;
}
namespace wmtk::components::multimesh::utils::detail {
std::string make_named_error_string(
    const std::string_view& path,
    const std::optional<uint8_t>& dimension,
    const std::optional<attribute::AttributeType>& type);
std::string make_named_error_string(const AttributeDescription& ad);
} // namespace wmtk::components::multimesh::utils::detail
