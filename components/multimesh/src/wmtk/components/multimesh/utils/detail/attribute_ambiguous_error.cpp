
#include "attribute_ambiguous_error.hpp"
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <algorithm>
#include <iterator>
#include "named_error_text.hpp"
namespace wmtk::components::multimesh::utils::detail {

std::string attribute_ambiguous_error::make_message(
    const AttributeDescription& ad,
    const std::vector<AttributeDescription>& possibilities)
{
    std::vector<std::string> names;

    std::string (*maker)(const AttributeDescription&) = make_named_error_string;
    std::transform(possibilities.begin(), possibilities.end(), std::back_inserter(names), maker);
    return fmt::format(
        "Multiple options for an attribute {} found: [{}]",
        make_named_error_string(ad),
        fmt::join(names, ","));
}
} // namespace wmtk::components::multimesh::utils::detail
