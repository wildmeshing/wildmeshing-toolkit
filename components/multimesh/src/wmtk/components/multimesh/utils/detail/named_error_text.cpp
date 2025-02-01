
#include "named_error_text.hpp"
#include <fmt/format.h>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <wmtk/attribute/AttributeType.hpp>
#include "../AttributeDescription.hpp"
namespace wmtk::components::multimesh::utils::detail {
std::string make_named_error_string(
    const std::string_view& path,
    const std::optional<uint8_t>& dimension,
    const std::optional<attribute::AttributeType>& type)
{
    std::string typestr;
    if (type.has_value()) {
        nlohmann::json j;
        j = type.value(); // just using the fact we can generate a strnig for this using json
                          // to get a printable string
        typestr = j;
    }
    if (type.has_value() && dimension.has_value()) {
        return fmt::format("named {} on {}-simplices of type {}", path, dimension.value(), typestr);
    } else if (dimension.has_value()) {
        return fmt::format("named {} on {}-simplices", path, dimension.value());
    } else if (type.has_value()) {
        return fmt::format("named {} of type {}", path, typestr);
    } else {
        return fmt::format("named {}", path);
    }
}
std::string make_named_error_string(const AttributeDescription& ad)
{
    return make_named_error_string(ad.path, ad.dimension, ad.type);
}
} // namespace wmtk::components::multimesh::utils::detail
